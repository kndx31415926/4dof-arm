function bus = get_bus_from_cfg(cfg, P_total_max_fallback)
% 从 cfg.BUS 组装 BUS 口径结构体；没有则给默认。
% 字段：
%   P_total_max  —— 总线正功上限（必须）
%   eta_mot      —— 电机正向效率（>0, ≤1）
%   eta_regen    —— 再生效率（>0, ≤1）
%   P_dump_max   —— 制动电阻消能上限（可 inf）
    if nargin < 2 || isempty(P_total_max_fallback)
        P_total_max_fallback = getfield_def(cfg,'P_TOTAL_MAX', inf);
    end
    bus = spinn3d_bus_defaults(struct('P_total_max', P_total_max_fallback));

    if isfield(cfg,'BUS') && isstruct(cfg.BUS)
        B = cfg.BUS;
        if isfield(B,'P_total_max') && ~isempty(B.P_total_max), bus.P_total_max = double(B.P_total_max); end
        if isfield(B,'eta_mot')     && ~isempty(B.eta_mot),     bus.eta_mot     = double(B.eta_mot); end
        if isfield(B,'eta_regen')   && ~isempty(B.eta_regen),   bus.eta_regen   = double(B.eta_regen); end
        if isfield(B,'P_dump_max')  && ~isempty(B.P_dump_max),  bus.P_dump_max  = double(B.P_dump_max); end
    end
end

% ===== local helper =====
function v = getfield_def(S,fn,def), if isstruct(S)&&isfield(S,fn)&&~isempty(S.(fn)), v=S.(fn); else, v=def; end, end
