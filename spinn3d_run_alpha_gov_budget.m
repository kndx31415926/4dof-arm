function out = spinn3d_run_alpha_gov_budget()
% 生成（governor-aware）数据集 → 分片写盘 → 主表聚合（不删列）→ 训练
% 规则：
%   - Pmax 与 payload：逐轨迹采样；单条轨迹内不变
%   - Ts 显式透传（取自 PB.time.Ts）
%   - 主表不做“删列”，训练/推理严格对齐

assert(exist('spinn3d_params_block','file')==2, '缺少 spinn3d_params_block.m');
PB = spinn3d_params_block();

% ===== 校验 =====
req(PB, {'paths','sample','power','alpha','caps','fric','alpha_budget','time','payload'});
req(PB.paths, {'root'});
req(PB.sample, {'dt'});
req(PB.time,   {'Ts'});
req(PB.alpha,  {'alpha_min','itmax'});
req(PB.power,  {'P_TOTAL_MAX'});
req(PB.fric,   {'B','Fc','vel_eps'});
req(PB.alpha_budget, {'n_traj','chunk_traj','pmax_mode','pmax_range', ...
                      'q0_range_deg','qref_range_deg', ...
                      'dls_lambda','dls_w_ori','bez_lift_z','bez_gamma','ns_min'});
req(PB.payload, {'range'});

N_TRAJ     = double(PB.alpha_budget.n_traj);
CHUNK_TRAJ = double(PB.alpha_budget.chunk_traj);
DT         = double(PB.sample.dt);
TS         = double(PB.time.Ts);
ALPHA_MIN  = double(PB.alpha.alpha_min);
ITMAX      = double(PB.alpha.itmax);

% caps + 总功率预算 + 可选 BUS 口径
caps0 = PB.caps; caps0.P_total_max = double(PB.power.P_TOTAL_MAX);
if isfield(PB,'bus') && isstruct(PB.bus)
    if isfield(PB.bus,'eta_share') && ~isempty(PB.bus.eta_share), caps0.eta_share=double(PB.bus.eta_share); end
    if isfield(PB.bus,'P_dump_max') && ~isempty(PB.bus.P_dump_max), caps0.P_dump_max=double(PB.bus.P_dump_max); end
end

% governor 参数
areg = PB.alpha_reg;   % 要求 PB.alpha_reg 完整
req(areg, {'beta','a_dot_up','a_dot_dn'});

% Robot
robot = resolve_robot_from_PB(PB); try, robot.DataFormat='row'; end

% 输出路径
SAVE_DIR = char(PB.paths.root);
if ~exist(SAVE_DIR,'dir'), mkdir(SAVE_DIR); end
chunk_pat = fullfile(SAVE_DIR, 'alpha_gov_ds_chunk_*.mat');
existing  = dir(chunk_pat);
used_ids  = parse_ids({existing.name}, 'alpha_gov_ds_chunk_(\\d+)\\.mat');
next_id   = max([0 used_ids]) + 1;

fprintf('[alpha-gov] Total %d traj, chunk=%d, out=%s | Ts=%.6f | payload, Pmax = per-trajectory\n', ...
        N_TRAJ, CHUNK_TRAJ, SAVE_DIR, TS);

% ===== 分片生成 =====
left = N_TRAJ; nChunks = ceil(N_TRAJ / CHUNK_TRAJ);
for c = 1:nChunks
    m = min(CHUNK_TRAJ, left); left = left - m;
    Xc = []; yc = [];

    for i = 1:m
        % 起终位形（deg → rad）
        q0d   = sample_box(PB.alpha_budget.q0_range_deg);
        qrefd = sample_box(PB.alpha_budget.qref_range_deg);
        q0 = deg2rad_local(q0d);  qf = deg2rad_local(qrefd);

        % Bézier + DLS-IK（几何域）
        cfgGeom = struct('DT',DT, ...
                         'DLS_LAMBDA', double(PB.alpha_budget.dls_lambda), ...
                         'DLS_W_ORI',  double(PB.alpha_budget.dls_w_ori), ...
                         'BEZ_LIFT_Z', double(PB.alpha_budget.bez_lift_z), ...
                         'BEZ_GAMMA',  double(PB.alpha_budget.bez_gamma), ...
                         'NS_MIN',     double(PB.alpha_budget.ns_min));
        TRAJ = make_traj_cart_bezier_dls_geom(robot, q0, qf, cfgGeom);

        % —— 本条轨迹的 P_total_max（单轨迹内固定）——
        [Pmax_use, caps_use] = sample_Pmax_once(PB, caps0);

        % —— 本条轨迹的 payload 向量（10 维，单轨迹内固定）——
        payload_struct = sample_payload_struct(PB.payload.range);

        % —— 生成 governor-aware DS（显式传 Ts + payload struct）——
        opts = struct('dt',DT, 'Ts',TS, 'alpha_min',ALPHA_MIN, 'itmax',ITMAX, ...
                      'fric', struct('B',double((PB.fric.B(:)).'), 'Fc',double((PB.fric.Fc(:)).'), 'vel_eps',double(PB.fric.vel_eps)), ...
                      'payload', payload_struct, ...
                      'BETA',double(areg.beta), 'A_DOT_UP',double(areg.a_dot_up), 'A_DOT_DN',double(areg.a_dot_dn));
        DSi = spinn3d_make_dataset_alpha_gov_geom(robot, TRAJ, caps_use, opts);

        Xc = [Xc; double(DSi.X)]; %#ok<AGROW>
        yc = [yc; double(DSi.y)]; %#ok<AGROW>
    end

    % 分片写盘（不删列；清洗已在 DSi 内完成）
    chunk_id = next_id + (c-1);
    fn_chunk = fullfile(SAVE_DIR, sprintf('alpha_gov_ds_chunk_%d.mat', chunk_id));
    save(fn_chunk, 'Xc','yc','caps0','PB','-v7.3');
    fprintf('[alpha-gov] wrote chunk #%d: %s (samples=%d)\n', chunk_id, fn_chunk, size(Xc,1));
end

% ===== 聚合主表（不删列）=====
files = dir(chunk_pat); assert(~isempty(files), '未生成任何分片；检查 PB.alpha_budget.* 是否齐全。');
Xagg=[]; yagg=[];
for i=1:numel(files)
    S = load(fullfile(files(i).folder, files(i).name), 'Xc','yc');
    Xagg = [Xagg; S.Xc]; %#ok<AGROW>
    yagg = [yagg; S.yc]; %#ok<AGROW>
end
Xagg(~isfinite(Xagg))=0;
yagg(~isfinite(yagg))=ALPHA_MIN; yagg = min(1.0, max(ALPHA_MIN, yagg));

master_mat = fullfile(SAVE_DIR, 'spinn3d_alpha_gov_dataset_master.mat');
X = Xagg; y = yagg; %#ok<NASGU>
save(master_mat, 'X','y','caps0','PB','-v7.3');
fprintf('[alpha-gov] 样本: %d | 主表: %s\n', size(X,1), master_mat);

% ===== 训练（严格对齐：不删列；只保存 mu/sig）=====
model_path = fullfile(SAVE_DIR, 'model_alpha_gov.mat');
spinn3d_train_alpha_gov(struct('X',Xagg,'y',yagg,'alpha_min',ALPHA_MIN), model_path);
out = struct('dataset_master', master_mat, 'model', model_path);
end

%% ============= helpers =============
function req(S, keys), if ischar(keys), keys={keys}; end
for i=1:numel(keys), assert(isfield(S,keys{i}) && ~isempty(S.(keys{i})), 'PB 缺少字段：%s', keys{i}); end
end
function q = sample_box(Qdeg), lo=Qdeg(:,1).'; hi=Qdeg(:,2).'; q=lo+(hi-lo).*rand(size(lo)); end
function ids = parse_ids(names, pat)
    ids = [];
    for i = 1:numel(names)
        m = regexp(names{i}, pat, 'tokens', 'once');
        if ~isempty(m), ids(end+1) = str2double(m{1}); end %#ok<AGROW>
    end
end
function robot = resolve_robot_from_PB(PB)
    robot = [];
    if isfield(PB,'robot') && isfield(PB.robot,'object') && ~isempty(PB.robot.object), robot = PB.robot.object; return; end
    if isfield(PB,'robot') && isfield(PB.robot,'builder') && ~isempty(PB.robot.builder), robot = feval(PB.robot.builder); return; end
    error('PB.robot 需要提供 object 或 builder。');
end
function y = deg2rad_local(x), y=(pi/180).*x; end
function [Pmax_use, caps_use] = sample_Pmax_once(PB, caps0)
    Pmode = lower(char(PB.alpha_budget.pmax_mode));
    Prng  = (double(PB.alpha_budget.pmax_range(:))).';
    switch Pmode
        case 'sample', Pmax_use = Prng(1) + rand()*(Prng(2)-Prng(1));
        case 'fixed',  Pmax_use = Prng(2);
        otherwise,     error('PB.alpha_budget.pmax_mode 只能为 fixed 或 sample');
    end
    caps_use = caps0; caps_use.P_total_max = Pmax_use;
end
function P = sample_payload_struct(R)
    m  = R.mass(1) + rand()*(R.mass(2)-R.mass(1));
    cx = R.com(1,1) + rand()*(R.com(1,2)-R.com(1,1));
    cy = R.com(2,1) + rand()*(R.com(2,2)-R.com(2,1));
    cz = R.com(3,1) + rand()*(R.com(3,2)-R.com(3,1));
    Lx = R.box_dims(1,1) + rand()*(R.box_dims(1,2)-R.box_dims(1,1));
    Ly = R.box_dims(2,1) + rand()*(R.box_dims(2,2)-R.box_dims(2,1));
    Lz = R.box_dims(3,1) + rand()*(R.box_dims(3,2)-R.box_dims(3,1));
    Ixx_c = m*(Ly^2+Lz^2)/12; Iyy_c = m*(Lx^2+Lz^2)/12; Izz_c = m*(Lx^2+Ly^2)/12;
    Ixx = Ixx_c + m*(cy^2+cz^2); Iyy = Iyy_c + m*(cx^2+cz^2); Izz = Izz_c + m*(cx^2+cy^2);
    Ixy = -m*cx*cy; Ixz = -m*cx*cz; Iyz = -m*cy*cz;
    P = struct('mass', m, 'com', [cx,cy,cz], 'inertia', [Ixx,Iyy,Izz,Ixy,Ixz,Iyz]);
end
