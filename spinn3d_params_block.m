function PB = spinn3d_params_block()
% ===== Central Param Block =====
% 所有参数集中在此处；其它脚本不得再造默认或兜底。
% 角度单位：度

%% Robot
PB.robot.object  = [];                    % 直接提供 rigidBodyTree（可留空）
PB.robot.builder = @spinn3d_build_robot;  % 构造函数句柄

%% Trajectory（demo/online 基准位形与名义限速）
PB.traj.q0_deg = [ 80;  80;  80;  80];
PB.traj.qf_deg = [-35; -20; -30; -80];
PB.traj.vmax   = 6;                     % [rad/s]
PB.traj.amax   = 12;                     % [rad/s^2]

%% Sampling / Time
PB.sample.dt = 0.01;                      % 几何采样步长（offline/预扫 α）
PB.time.Ts   = 1e-3;                      % 控制/仿真步长

%% Power / Alpha
PB.power.P_TOTAL_MAX = 20;                % 总正功上限 [W]（只计正功）
PB.alpha.alpha_min   = 0.15;              % α 下界
PB.alpha.itmax       = 12;                % 可行化二分迭代

%% Limits（CAPS）
PB.caps.tau_max    = 60*ones(4,1);
PB.caps.qd_max     = 3.0*ones(4,1);
PB.caps.qdd_max    = 6.0*ones(4,1);
PB.caps.P_axis_max = inf(4,1);
PB.caps.countRegen = false;

%% Controller gains
PB.control.Kp = 40;
PB.control.Kd = 8;

%% Stop（demo/online）
PB.stop.ee_tol      = 0.02;               % [m]
PB.stop.dq_tol_rad  = 0.03;               % [rad/s]
PB.stop.stable_time = 0.10;               % [s]

%% Friction（训练/在线统一口径）
PB.fric.B       = [0.05 0.05 0.03 0.02];  % 粘滞 [N·m·s/rad]（1×nJ 行向量）
PB.fric.Fc      = [0.4  0.3  0.2  0.1 ];  % 库伦 [N·m]
PB.fric.vel_eps = 1e-3;                   % sign 平滑阈值

%% Measurement / torque noise（demo/online）
PB.noise.sigma_q  = 1e-4;                 % [rad]
PB.noise.sigma_dq = 1e-4;                 % [rad/s]
PB.noise.tau_std  = 1e-4;                 % [N·m]

%% Paths（使用 char + fullfile，避免 "…" 与 + ）
PB.paths.root           = 'data_alpha';
PB.paths.dataset_master = fullfile(PB.paths.root, 'spinn3d_alpha_dataset_master.mat');
PB.paths.dataset_table  = fullfile(PB.paths.root, 'spinn3d_alpha_dataset_table_master.mat');
PB.paths.model_alpha    = fullfile(PB.paths.root, 'model_alpha_gov.mat');

%% Train（是否在生成完数据后立刻训练）
PB.train.enable = true;
PB.train.epochs = 25;
PB.train.batch  = 512;
PB.train.lr     = 1e-3;

%% Payload（demo 固定值 + 训练采样范围）
PB.payload.mode = 'overwrite_last';              % 'overwrite_last' | 'attach_tool'
PB.payload.demo.mass    = 0.40;                  % [kg]
PB.payload.demo.com     = [0 0 0.05];            % [m]
PB.payload.demo.inertia = [0.0008 0.0008 0.0006 0 0 0]; % [Ixx Iyy Izz Ixy Ixz Iyz]
PB.payload.demo.mount.xyz     = [0 0 0];
PB.payload.demo.mount.rpy_deg = [0 0 0];

PB.payload.range.mass     = [0.20 0.80];
PB.payload.range.com      = [-0.03 0.03; -0.03 0.03; 0.00 0.10];
PB.payload.range.box_dims = [0.03 0.06; 0.03 0.06; 0.03 0.08];
% 如需固定训练端 payload 向量，可加：
% PB.payload.train.mass    = 0.40;
% PB.payload.train.com     = [0 0 0.05];
% PB.payload.train.inertia = [0.0008 0.0008 0.0006 0 0 0];

%% ===== Alpha 数据集预算（run 脚本仅认这一组；缺一即报错）=====
PB.alpha_budget.n_traj      = 50;               % 总轨迹数
PB.alpha_budget.chunk_traj  = 50;                % 每分片轨迹数
PB.alpha_budget.pmax_mode   = 'fixed';           % 'fixed' | 'sample'
PB.alpha_budget.pmax_range  = [PB.power.P_TOTAL_MAX PB.power.P_TOTAL_MAX]; % [Pmin Pmax]
PB.alpha_budget.q0_range_deg   = [-30 30; -20 20; -20 20; -20 20];        % nJ×2（度）
PB.alpha_budget.qref_range_deg = [ 15 45;  10 30; -40 -10;  10 40];       % nJ×2（度）
PB.alpha_budget.dls_lambda  = 5e-3;
PB.alpha_budget.dls_w_ori   = 0.02;
PB.alpha_budget.bez_lift_z  = 0.15;
PB.alpha_budget.bez_gamma   = 1/3;
PB.alpha_budget.ns_min      = 200;

%% Governor / alpha regulator（严格必填）
PB.alpha_reg.beta    = 0.85;   % 一阶滤波系数 BETA，取 [0,1)，越大越“粘”
PB.alpha_reg.a_dot_up = 2.5;   % α 上升速率上限 [1/s]
PB.alpha_reg.a_dot_dn = 5.0;   % α 下降速率上限 [1/s]

end
