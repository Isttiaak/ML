function d = mopso_crowding(F, normalize)
% MOPSO_CROWDING  NSGA-II crowding distance of each row of F (n x m).
%   d = mopso_crowding(F, normalize)
%   normalize (default true): each objective's gaps are divided by its range
%   over F, so objectives in different units contribute equally. Boundary
%   points of an objective get Inf. Objectives with zero range are skipped,
%   so an all-duplicate set returns all zeros. n <= 2 returns Inf.
if nargin < 2
    normalize = true;
end
[n, m] = size(F);
d = zeros(n, 1);
if n <= 2
    d = Inf(n, 1);
    return;
end
for j = 1:m
    [s, ord] = sort(F(:, j));
    r = s(end) - s(1);
    if ~(r > 0) || ~isfinite(r)
        continue;
    end
    d(ord(1)) = Inf;
    d(ord(end)) = Inf;
    gap = s(3:end) - s(1:end-2);
    if normalize
        gap = gap / r;
    end
    d(ord(2:end-1)) = d(ord(2:end-1)) + gap;
end
end
