function ctl = spinn3d_controller_ctcgov(robot, traj, gains, limits, opts)
% CTC + Governor（严格版；母线功率口径）
% - NN-α：dlnetwork.forward + governor 特征；标准化前按 keepBaseIdx 压基础特征；
% - 尾巴 6 维固定顺序：[a_prev,BETA,A_DOT_UP,A_DOT_DN,Ts,s_norm]（不压列）。

try, robot.DataFormat='row'; end
nJ = local_num_joints(robot);

% ===== 基本参数（无默认）=====
assert(isfield(opts,'Ts') && opts.Ts>0, 'opts.Ts 必须提供且 >0');
Ts  = double(opts.Ts);
Kp  = vec_row(reqf(gains,'Kp'), nJ);
Kd  = vec_row(reqf(gains,'Kd'), nJ);

% ===== NN α（严格）=====
assert(isfield(opts,'nnAlpha') && isstruct(opts.nnAlpha), '缺少 opts.nnAlpha');
nn = opts.nnAlpha;
assert(isfield(nn,'enable') && nn.enable==true, 'nnAlpha 未启用');
assert(isfield(nn,'model') && isfield(nn.model,'net') && isfield(nn.model,'preproc'), 'nnAlpha.model 不完整');
assert(isa(nn.model.net,'dlnetwork'), 'nn.model.net 必须为 dlnetwork');
assert(isfield(nn,'featureFcn') && isa(nn.featureFcn,'function_handle'), '必须提供 governor 版特征函数');
assert(isfield(nn,'payload') && isstruct(nn.payload), 'nnAlpha.payload 必须为 struct(mass/com/inertia)');

% α 正则
areg = struct('A_DOT_UP',2.5, 'A_DOT_DN',5.0, 'BETA',0.85, 'N_BISECT',3, 'GAMMA',1);
if isfield(opts,'alphaReg') && isstruct(opts.alphaReg)
    fns = fieldnames(areg);
    for i=1:numel(fns)
        if isfield(opts.alphaReg, fns{i}) && ~isempty(opts.alphaReg.(fns{i}))
            areg.(fns{i}) = double(opts.alphaReg.(fns{i}));
        end
    end
end

% 母线/摩擦/限值
bus  = spinn3d_bus_defaults(getfield_def(opts,'bus',struct()));
fric = getfield_def(opts,'fric', struct('B',0,'Fc',0,'vel_eps',1e-3));
caps = normalize_caps(limits, nJ);
Pcap = getfield_def(limits,'P_total_max', inf);

% 状态
state = struct('a_prev', getfield_def(nn,'alpha_floor',0));

% ===== 主循环（注意：8 参，包含几何进度 s） =====
    function [tau_cmd, info] = step(t, q_meas, dq_meas, q_ref, dq_ref, ddq_ref, robot_in, s) %#ok<INUSD>
        q_meas = row(q_meas); dq_meas = row(dq_meas);
        q_ref  = row(q_ref);  dq_ref  = row(dq_ref);  ddq_ref = row(ddq_ref);

        % --- 1) 特征：基础 + payload10 + 尾巴6（严格顺序）---
        s_norm = min(1.0, s / max(traj.Tf, 1e-9));   % ★ 用几何进度，和训练对齐
        x_full = nn.featureFcn(q_ref, dq_ref, ddq_ref, robot, nn.payload, Pcap, caps, ...
                               state.a_prev, areg.BETA, areg.A_DOT_UP, areg.A_DOT_DN, Ts, s_norm);
        x_full = row(double(x_full));

        % --- 2) 仅压基础特征，尾巴6保留：按 keepBaseIdx ---
        assert(isfield(nn.model.preproc,'keepBaseIdx') && ~isempty(nn.model.preproc.keepBaseIdx), ...
               '模型缺 preproc.keepBaseIdx（训练端未保存列掩码）');
        kb    = logical(nn.model.preproc.keepBaseIdx(:).');
        Dtot  = numel(x_full);
        Dtail = 6;                % 固定：a_prev,BETA,A_UP,A_DN,Ts,s_norm
        Dbase = Dtot - Dtail;
        assert(numel(kb)==Dbase, 'keepBaseIdx 长度(%d)≠基础特征维(%d)', numel(kb), Dbase);

        x_base = x_full(1:Dbase);
        x_tail = x_full(Dbase+1:end);
        x      = [x_base(kb), x_tail];

        % --- 3) z‑score + NN 前向 ---
        mu  = row(double(nn.model.preproc.mu));
        sig = row(double(nn.model.preproc.sig));
        assert(numel(mu)==numel(x) && numel(sig)==numel(x), ...
               '特征维(%d)与 mu/sig 维(%d)不一致', numel(x), numel(mu));
        z   = (x - mu) ./ max(sig, 1e-12);
        dlX = dlarray(single(z'), 'CB');
        a0  = clamp(double(extractdata(forward(nn.model.net, dlX))), getfield_def(nn,'alpha_floor',0), 1.0);

        % --- 4) Governor 护栏（BUS 口径）---
        [a, a_slew, a_filt, adot, guard] = alpha_guard_backtrack( ...
            a0, state.a_prev, Ts, q_ref, dq_ref, ddq_ref, robot, caps, Pcap, fric, areg, bus);

        % --- 5) CTC（含 α̇）---
        dq_cmd  = a    * dq_ref;
        ddq_cmd = a*a  * ddq_ref + adot * dq_ref;

        e  = q_ref  - q_meas;
        ed = dq_cmd - dq_meas;
        v  = ddq_cmd + Kd.*ed + Kp.*e;

        Mq = massMatrix(robot, q_meas);
        c  = velocityProduct(robot, q_meas, dq_meas);
        g  = gravityTorque(robot, q_meas);
        tau_cmd = (Mq * v(:) + c(:) + g(:)).';

        if any(fric.B) || any(fric.Fc)
            tau_cmd = tau_cmd + fric.B.*dq_meas + fric.Fc.*tanh(dq_meas./max(fric.vel_eps,1e-9));
        end

        state.a_prev = a;
        info = struct('a_max',a,'a0',a0,'guard',guard,'a_slew',a_slew,'a_filt',a_filt,'adot',adot,'mode','nn');
    end

ctl = struct('step',@step, 'Kp',Kp,'Kd',Kd,'limits',caps,'nnAlpha',nn,'Ts',Ts,'alphaReg',areg,'bus',bus);
end

% ================= 辅助函数 =================
function n = local_num_joints(robot)
    n = numel(homeConfiguration(robot));
end
function caps = normalize_caps(lim, nJ)
    caps = struct();
    fn = {'tau_max','qd_max','qdd_max','P_axis_max'};
    for i=1:numel(fn)
        if isfield(lim,fn{i}) && ~isempty(lim.(fn{i}))
            v = lim.(fn{i})(:).'; if isscalar(v), v = repmat(v,1,nJ); end
            caps.(fn{i}) = double(v(1:nJ));
        else
            caps.(fn{i}) = inf(1,nJ);
        end
    end
    if isfield(lim,'countRegen'), caps.countRegen = logical(lim.countRegen); end
end
function v = vec_row(x, n), x=double(x); if isscalar(x), v=repmat(x,1,n); else, v=x(:).'; end, v=v(1:n); end
function v = reqf(S, k), assert(isfield(S,k) && ~isempty(S.(k)), ['缺少字段: ',k]); v = S.(k); end
function x = getfield_def(S, k, d), if isstruct(S)&&isfield(S,k)&&~isempty(S.(k)), x=S.(k); else, x=d; end, end
function r = row(v), r = v(:)'; end
function y = clamp(x, lo, hi), y = min(hi, max(lo, x)); end
function m = min_safe(v), m = min(v(:)); if isempty(m), m = 1.0; end, end

function [a, a_slew, a_filt, adot, G] = alpha_guard_backtrack(a0, a_prev, Ts, q, dq_ref, ddq_ref, robot, caps, Pcap, fric, areg, bus) %#ok<INUSD>
    a0     = clamp(a0, 0, 1);
    a_slew = min(a0, a_prev + areg.A_DOT_UP*Ts);
    a_slew = max(a_slew, a_prev - areg.A_DOT_DN*Ts);
    a_filt = areg.BETA*a_prev + (1-areg.BETA)*a_slew;

    sf_qd  = min_safe(caps.qd_max  ./ max(abs(dq_ref),  1e-12));
    sf_qdd = min_safe(sqrt(caps.qdd_max ./ max(abs(ddq_ref),1e-12)));
    a_kin  = min([1.0, sf_qd, sf_qdd]);

    a    = min(a_filt, a_kin);
    adot = (a - a_prev)/max(Ts,1e-12);
    G = struct('a0',a0,'a_slew',a_slew,'a_filt',a_filt,'a_kin',a_kin);
end
