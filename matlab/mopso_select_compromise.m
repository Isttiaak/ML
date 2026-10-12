function [idx, score] = mopso_select_compromise(F, weights, method)
% MOPSO_SELECT_COMPROMISE  Pick one compromise solution from a Pareto set.
%   [idx, score] = mopso_select_compromise(F, weights, method)
%   F       : n x m objective matrix (minimisation).
%   weights : 1 x m non-negative weights (default equal); normalised to sum 1.
%   method  : 'ideal_point' (default) min-max normalise each objective to
%                 [0,1] over F, score = ||w .* Fn||, i.e. weighted Euclidean
%                 distance to the ideal point (all zeros after normalisation).
%             'origin_maxnorm' original notebook rule: Fn = F ./ (max(F)+1e-12),
%                 score = ||w .* Fn|| (distance to the origin, not the ideal
%                 point; biased when an objective's minimum is far from 0).
%             'knee' (2 objectives) maximum distance below the straight line
%                 joining the two extreme points in normalised space;
%                 weights are ignored. Falls back to 'ideal_point' for m ~= 2
%                 or when the front is degenerate (fewer than 3 distinct
%                 points or no point below the line).
%   idx   : row index of the chosen solution (argmin of score, first on ties).
%   score : n x 1, lower is better (for 'knee', score = -distance below line).
%   Constant objective columns (range 0) are ignored in the normalised methods.
%   Rows containing non-finite values get score Inf.
if nargin < 3 || isempty(method)
    method = 'ideal_point';
end
[n, m] = size(F);
if n == 0
    idx = [];
    score = zeros(0, 1);
    return;
end
if nargin < 2 || isempty(weights)
    weights = ones(1, m);
end
w = double(weights(:).');
if numel(w) ~= m
    error('mopso_select_compromise:weights', 'weights must have %d elements', m);
end
if any(w < 0) || sum(w) <= 0
    error('mopso_select_compromise:weights', 'weights must be non-negative with a positive sum');
end
w = w / sum(w);
bad = any(~isfinite(F), 2);

switch lower(method)
    case 'ideal_point'
        score = local_ideal(F, w, bad);
    case 'origin_maxnorm'
        Fn = F ./ (max(F, [], 1) + 1e-12);
        score = sqrt(sum((Fn .* w).^2, 2));
    case 'knee'
        if m ~= 2
            score = local_ideal(F, w, bad);
        else
            score = local_knee(F, w, bad);
        end
    otherwise
        error('mopso_select_compromise:method', 'unknown method ''%s''', method);
end
score(bad) = Inf;
[~, idx] = min(score);
end

function score = local_ideal(F, w, bad)
Fn = mopso_minmax_norm(F(~bad, :));
score = Inf(size(F, 1), 1);
score(~bad) = sqrt(sum((Fn .* w).^2, 2));
end

function score = local_knee(F, w, bad)
good = find(~bad);
Fn = mopso_minmax_norm(F(good, :));
U = unique(Fn, 'rows');
if size(U, 1) < 3
    score = local_ideal(F, w, bad);
    return;
end
% extremes: best f1 (ties -> best f2) and best f2 (ties -> best f1)
[~, ia] = sortrows(Fn, [1 2]);
[~, ib] = sortrows(Fn, [2 1]);
A = Fn(ia(1), :);
B = Fn(ib(1), :);
v = B - A;
L = sqrt(sum(v.^2));
if L <= 0
    score = local_ideal(F, w, bad);
    return;
end
% signed distance; positive = on the ideal-point side of the line
nrm = [v(2), -v(1)] / L;
if (([0 0] - A) * nrm.') < 0
    nrm = -nrm;
end
dist = (Fn - A) * nrm.';
if max(dist) <= 1e-12
    score = local_ideal(F, w, bad);
    return;
end
score = Inf(size(F, 1), 1);
score(good) = -dist;
end
