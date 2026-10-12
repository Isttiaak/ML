function mask = mopso_nondominated(F)
% MOPSO_NONDOMINATED  Logical mask of Pareto non-dominated rows (minimisation).
%   Row a dominates row b if all(a <= b) and any(a < b). Exact duplicates do
%   not dominate each other, so both are kept (use unique/dedupe separately).
n = size(F, 1);
mask = true(n, 1);
for i = 1:n
    if ~mask(i)
        continue;
    end
    fi = F(i, :);
    dom = all(F <= fi, 2) & any(F < fi, 2);
    if any(dom)
        mask(i) = false;
    end
end
end
