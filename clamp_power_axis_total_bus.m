function tau_out = clamp_power_axis_total_bus(tau, dq, P_axis_max, bus)
% 命令级扭矩护栏（BUS 口径）：
%  1) 逐轴正功限幅：P_i = max(tau_i*dq_i, 0) <= P_axis_max(i)
%  2) 总线正功限幅：sum_i P_i <= bus.P_total_max
% 负功（再生）不在命令侧限幅，按 BUS 口径在功率统计里处理。

    tau = row(tau); dq = row(dq);
    n   = numel(tau);
    if nargin < 3 || isempty(P_axis_max), P_axis_max = inf(1,n); end
    P_axis_max = row(P_axis_max);

    % --- 逐轴正功限幅 ---
    Paxis = tau .* dq;
    for i = 1:n
        if isfinite(P_axis_max(i)) && Paxis(i) > 0 && Paxis(i) > P_axis_max(i)
            s = P_axis_max(i) / max(Paxis(i), 1e-12);
            tau(i) = tau(i) * s;
        end
    end

    % --- 总线正功限幅 ---
    Paxis = tau .* dq;
    Ppos  = sum(max(Paxis, 0));
    Pcap  = getfield_def(bus, 'P_total_max', inf);
    if Ppos > Pcap
        s = Pcap / max(Ppos, 1e-12);
        pos = (Paxis > 0);
        tau(pos) = tau(pos) * s;   % 只缩放正功通道
    end

    tau_out = tau;
end

% ===== local helpers =====
function y = row(x), y = x(:)'; end
function v = getfield_def(S,fn,def), if isstruct(S)&&isfield(S,fn)&&~isempty(S.(fn)), v=S.(fn); else, v=def; end, end
