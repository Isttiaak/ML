function [G, KPV, KIV] = drv_decode(X, ctrl, cfg)
%DRV_DECODE  Split MOPSO decision rows into controller gains and outer gains.
%   [G, KPV, KIV] = drv_decode(X, ctrl, cfg)
%   X   n x ndim (layout of drv_search_space)
%   G   n x ngains (sim_boost order);  KPV, KIV  n x 1 (P.KPV / P.KIV when
%   the outer loop is not tuned)
S = drv_search_space(ctrl, cfg);
if size(X, 2) ~= numel(S.lb)
    error('drv_decode:dim', '%s expects %d decision variables, got %d', S.ctrl, numel(S.lb), size(X, 2));
end
n = size(X, 1);
G = X(:, 1:S.ngains);
if S.outer
    KPV = X(:, S.ngains + 1);
    KIV = X(:, S.ngains + 2);
else
    KPV = repmat(cfg.P.KPV, n, 1);
    KIV = repmat(cfg.P.KIV, n, 1);
end
end
