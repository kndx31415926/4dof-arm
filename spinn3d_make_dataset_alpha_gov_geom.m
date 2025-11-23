function DS = spinn3d_make_dataset_alpha_gov_geom(robot, TRAJ, caps, opts)
% 几何轨迹 → governor-aware 数据集
% X = α++ + payload10 + [a_prev, BETA, AUP, ADN, Ts, s_norm]
% y = a_des_teacher（经 governor 后应落在 a*）

    assert(isfield(opts,'dt') && opts.dt>0, 'opts.dt 必须提供且 > 0');
    assert(isfield(opts,'alpha_min'), 'opts.alpha_min 必须提供');
    assert(isfield(opts,'itmax'), 'opts.itmax 必须提供');
    assert(isfield(opts,'fric'), 'opts.fric 必须提供');

    dt   = double(opts.dt);
    Ts   = double(getfield_def(opts, 'Ts', dt));  % ★ 正确使用控制周期
    beta = getfield_def(opts,'BETA',0.85);
    up   = getfield_def(opts,'A_DOT_UP',2.5);
    dn   = getfield_def(opts,'A_DOT_DN',5.0);
    Pmax = double(caps.P_total_max);
    fric = opts.fric;

    % payload：兼容 struct 或 1x10
    if ~isfield(opts,'payload') || isempty(opts.payload)
        payload = struct('mass',0,'com',[0 0 0],'inertia',zeros(1,6));
    else
        if isstruct(opts.payload)
            payload = opts.payload;
        else
            v = double(opts.payload(:)).'; v = [v, zeros(1,10-numel(v))];
            payload = struct('mass',v(1),'com',v(2:4),'inertia',v(5:10));
        end
    end

    % s 采用几何步长 dt 采样；Ts 专用于 governor/特征
    s_grid = 0:dt:TRAJ.Tf; if s_grid(end)<TRAJ.Tf, s_grid=[s_grid, TRAJ.Tf]; end

    X = []; Y = []; a_prev = double(opts.alpha_min); k = 1;
    for si = 1:numel(s_grid)
        s = s_grid(si);
        [q, dqg, ddqg] = TRAJ.eval(s);  q=row(q); dqg=row(dqg); ddqg=row(ddqg);

        % teacher：可行 α → 反解 governor 得 a_des
        a_star = alpha_feasible(q, dqg, ddqg, 1.0, robot, caps, Pmax, opts.alpha_min, opts.itmax, fric);
        a_des  = alpha_des_teacher(a_star, a_prev, beta, up, dn, Ts);

        % 特征：s_norm 必须是几何进度（与训练一致）
        s_norm = min(1.0, s/max(TRAJ.Tf,1e-9));
        xb = spinn3d_features_alpha_plus_payload(q,dqg,ddqg, robot, payload, Pmax, caps);
        x  = double([xb, a_prev, beta, up, dn, Ts, s_norm]);

        X(k,:) = x; Y(k,1) = a_des; %#ok<AGROW>
        a_prev = governor_step(a_des, a_prev, beta, up, dn, Ts);  % 取第一返回值 a_post
        k = k + 1;
    end

    % 清洗
    X(~isfinite(X)) = 0;
    Y(~isfinite(Y)) = opts.alpha_min;
    Y = min(1.0, max(opts.alpha_min, Y));

    DS = struct('X',X,'y',Y,'alpha_min',opts.alpha_min);
end

function v=row(v), v=v(:)'; end
function x = getfield_def(S,fn,def), if isstruct(S)&&isfield(S,fn)&&~isempty(S.(fn)), x=S.(fn); else, x=def; end, end
