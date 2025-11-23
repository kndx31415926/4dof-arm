function [Ppos, Pneg, Pgrid, Pdump, Paxis] = spinn3d_bus_power(tau, dq, bus)
% 计算 BUS 口径功率：
%  Paxis = tau .* dq；
%  Ppos = sum(max(Paxis,0))，Pneg = sum(min(Paxis,0))；
%  Pgrid = Ppos/eta_mot - eta_regen*(-Pneg)，若超过总线上限则超出记入 Pdump。

    tau   = row(tau); dq = row(dq);
    Paxis = tau .* dq;

    Ppos = sum(max(Paxis, 0));
    Pneg = sum(min(Paxis, 0));   % ≤ 0

    eta_mot   = getfield_def(bus,'eta_mot',   1.0);
    eta_regen = getfield_def(bus,'eta_regen', 1.0);
    Pcap      = getfield_def(bus,'P_total_max', inf);
    Pdump_max = getfield_def(bus,'P_dump_max', inf);

    Pgrid_raw = Ppos/max(eta_mot,1e-12) - eta_regen * (-Pneg);

    if Pgrid_raw > Pcap
        overflow = Pgrid_raw - Pcap;
        Pdump = min(overflow, Pdump_max);
        Pgrid = Pgrid_raw - Pdump;   % = min(Pgrid_raw, Pcap+Pdump_max)
    else
        Pgrid = Pgrid_raw;
        Pdump = 0;
    end
end

% ===== local helpers =====
function y = row(x), y = x(:)'; end
function v = getfield_def(S,fn,def), if isstruct(S)&&isfield(S,fn)&&~isempty(S.(fn)), v=S.(fn); else, v=def; end, end
