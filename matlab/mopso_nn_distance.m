function d = mopso_nn_distance(F, normalize)
% MOPSO_NN_DISTANCE  Euclidean distance from each row of F to its nearest other row.
%   normalize (default false) min-max normalises the objectives first. With
%   normalize=false this is the density measure of the original notebook
%   (raw objective units). A single row returns Inf; duplicates return 0.
if nargin < 2
    normalize = false;
end
n = size(F, 1);
if n <= 1
    d = Inf(n, 1);
    return;
end
if normalize
    F = mopso_minmax_norm(F);
end
D2 = zeros(n, n);
for j = 1:size(F, 2)
    c = F(:, j);
    D2 = D2 + (c - c.').^2;
end
D2(1:n+1:end) = Inf;
d = sqrt(min(D2, [], 2));
end
