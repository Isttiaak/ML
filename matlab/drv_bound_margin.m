function marg = drv_bound_margin(X, S)
%DRV_BOUND_MARGIN  Distance of decision rows to the nearest bound.
%   marg = drv_bound_margin(X, S)   S from drv_search_space
%   marg (n x ndim) = min(z - zlb, zub - z) / (zub - zlb) in the search
%   space (log10 for log_scale dims): 0 = on a bound, 0.5 = centre.
z = X;
zl = S.lb;
zu = S.ub;
lg = S.log_scale;
z(:, lg) = log10(X(:, lg));
zl(lg) = log10(zl(lg));
zu(lg) = log10(zu(lg));
rng_ = zu - zl;
rng_(rng_ <= 0) = 1;
marg = min(z - zl, zu - z) ./ rng_;
marg = max(marg, 0);
end
