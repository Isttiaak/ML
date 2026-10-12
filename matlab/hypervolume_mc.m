function [hv, se] = hypervolume_mc(F, ref, nsamples, seed, method)
% HYPERVOLUME_MC  Hypervolume (minimisation) dominated by F and bounded by ref.
%   [hv, se] = hypervolume_mc(F, ref, nsamples, seed, method)
%   F        : n x m objective vectors; ref : 1 x m reference (nadir) point.
%   nsamples : Monte-Carlo samples (default 1e5).  seed : default 1.
%   method   : 'auto' (default; exact for m <= 2, Monte Carlo otherwise),
%              'mc' (force Monte Carlo), 'exact' (m <= 2 only).
%   se       : standard error of the estimate (0 for the exact path).
%   Points not strictly better than ref in every objective contribute 0.
%   Sampling box: [min(F), ref]. The global RNG state is saved and restored,
%   so calling this function does not change the caller's random stream.
if nargin < 3 || isempty(nsamples)
    nsamples = 1e5;
end
if nargin < 4 || isempty(seed)
    seed = 1;
end
if nargin < 5 || isempty(method)
    method = 'auto';
end
ref = double(ref(:).');
m = numel(ref);
if size(F, 2) ~= m
    error('hypervolume_mc:dim', 'F has %d columns but ref has %d', size(F, 2), m);
end
se = 0;
keep = all(isfinite(F), 2) & all(F < ref, 2);
F = F(keep, :);
if isempty(F)
    hv = 0;
    return;
end
F = F(mopso_nondominated(F), :);
F = unique(F, 'rows');

use_exact = strcmpi(method, 'exact') || (strcmpi(method, 'auto') && m <= 2);
if use_exact
    if m > 2
        error('hypervolume_mc:exact', 'exact path only implemented for <= 2 objectives');
    end
    if m == 1
        hv = ref - min(F);
        return;
    end
    F = sortrows(F, [1 2]);
    hv = 0;
    prev = ref(2);
    for i = 1:size(F, 1)
        if F(i, 2) < prev
            hv = hv + (ref(1) - F(i, 1)) * (prev - F(i, 2));
            prev = F(i, 2);
        end
    end
    return;
end

lo = min(F, [], 1);
span = ref - lo;
vol = prod(span);
saved = rng;
rng(seed, 'twister');
chunk = 20000;
done = 0;
hits = 0;
np = size(F, 1);
while done < nsamples
    k = min(chunk, nsamples - done);
    U = lo + rand(k, m) .* span;
    dom = false(k, 1);
    for i = 1:np
        dom = dom | all(U >= F(i, :), 2);
    end
    hits = hits + sum(dom);
    done = done + k;
end
rng(saved);
p = hits / nsamples;
hv = vol * p;
se = vol * sqrt(p * (1 - p) / nsamples);
end
