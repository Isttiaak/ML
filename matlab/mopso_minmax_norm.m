function [Fn, fmin, frange] = mopso_minmax_norm(F)
% MOPSO_MINMAX_NORM  Column-wise min-max normalisation to [0,1].
%   Constant columns (range 0) map to 0 so they never influence distances.
%   Non-finite entries are ignored when computing min/max.
[n, m] = size(F);
fmin = zeros(1, m);
frange = zeros(1, m);
Fn = zeros(n, m);
for j = 1:m
    col = F(:, j);
    ok = isfinite(col);
    if any(ok)
        lo = min(col(ok));
        hi = max(col(ok));
    else
        lo = 0;
        hi = 0;
    end
    fmin(j) = lo;
    frange(j) = hi - lo;
    if frange(j) > 0
        Fn(:, j) = (col - lo) / frange(j);
    else
        Fn(:, j) = zeros(n, 1);
        Fn(~ok, j) = Inf;
    end
end
end
