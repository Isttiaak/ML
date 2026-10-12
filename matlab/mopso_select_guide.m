function [idx, fallback] = mopso_select_guide(dens)
% MOPSO_SELECT_GUIDE  Roulette-wheel choice of a leader, weight = density measure.
%   [idx, fallback] = mopso_select_guide(dens)
%   dens: crowding or nearest-neighbour distance of each archive member
%   (larger = less crowded = more likely). Inf entries (boundary points) get
%   the largest finite weight. If no entry is finite and positive (all zero,
%   all Inf, NaN) a uniform choice is made instead (among Inf entries if any)
%   and fallback = true. The original notebook raised an error in this case.
%   Exactly one rand() draw per call, whatever the branch.
d = dens(:);
n = numel(d);
r = rand;
fallback = false;
if n <= 1
    idx = 1;
    return;
end
d(isnan(d) | d < 0) = 0;
finpos = isfinite(d) & d > 0;
if any(finpos)
    d(isinf(d)) = max(d(finpos));
    c = cumsum(d);
    idx = find(c > r * c(end), 1, 'first');
    if isempty(idx)
        idx = find(d > 0, 1, 'last');
    end
else
    fallback = true;
    cand = find(isinf(d));
    if isempty(cand)
        cand = (1:n).';
    end
    k = min(numel(cand), floor(r * numel(cand)) + 1);
    idx = cand(k);
end
end
