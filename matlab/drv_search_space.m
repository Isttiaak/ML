function S = drv_search_space(ctrl, cfg)
%DRV_SEARCH_SPACE  Decision variables of one controller for the MOPSO study.
%   S = drv_search_space(ctrl, cfg)   cfg from study_config
%   S.names, S.lb, S.ub, S.log_scale (1 x ndim): controller gains in the
%   sim_boost order, followed by KPV, KIV when cfg.tune_outer and the
%   controller has the outer voltage loop.
%   S.ngains  number of controller gains;  S.outer  true if KPV/KIV appended
%   S.x0      baseline row (P.gains_baseline + P.KPV/P.KIV) clipped to bounds
[name, ~, ng, has_outer] = drv_ctrl_name(ctrl);
v = cfg.vars.(name);
S.ctrl = name;
S.ngains = ng;
S.outer = logical(cfg.tune_outer) && has_outer;
S.names = v.names(:).';
S.lb = double(v.lb(:).');
S.ub = double(v.ub(:).');
S.log_scale = logical(v.log_scale(:).');
x0 = double(cfg.P.gains_baseline.(name)(:).');
if S.outer
    o = cfg.outer_vars;
    S.names = [S.names, o.names(:).'];
    S.lb = [S.lb, double(o.lb(:).')];
    S.ub = [S.ub, double(o.ub(:).')];
    S.log_scale = [S.log_scale, logical(o.log_scale(:).')];
    x0 = [x0, cfg.P.KPV, cfg.P.KIV];
end
S.x0 = min(max(x0, S.lb), S.ub);
S.x0_clipped = any(S.x0 ~= x0);
end
