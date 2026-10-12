function [arch_X, arch_F, best, info] = mopso(objfun, lb, ub, opts)
% MOPSO  Multi-objective particle swarm optimisation (minimisation).
%   [arch_X, arch_F, best, info] = mopso(objfun, lb, ub, opts)
%
%   objfun : function handle. opts.vectorized = false (default): f = objfun(x)
%            with x a 1 x ndim row, f a 1 x nobj row. opts.vectorized = true:
%            F = objfun(X) with X n_particles x ndim, F n_particles x nobj
%            (one call per iteration, e.g. a Simulink parsim batch).
%            x is always in the user's units (log_scale only changes the search).
%   lb, ub : 1 x ndim bounds.
%   opts   : struct, every field optional:
%     n_particles (20)  n_iter (30)  seed (42)  archive_size (60)
%     w (0.6; or [w_start w_end] for a linear decrease)  c1 (1.5)  c2 (1.5)
%     log_scale (false; logical 1 x ndim, search those dims in log10 space)
%     vel_max_frac (0.2; velocity clamp as fraction of the search range, Inf = off)
%     bound_handling ('reset_velocity' clip + zero that velocity component,
%                     'reflect' mirror position + reverse velocity, 'clip' legacy)
%     normalize_objectives (true; density measures on min-max normalised objectives)
%     density ('crowding' NSGA-II crowding distance, 'nn' nearest-neighbour
%              distance) - used for leader selection (and pruning if 'nn')
%     guide_selection ('roulette' on density, 'tournament' binary on density,
%              'uniform'); all-zero/all-Inf densities fall back to uniform
%     prune ('pair' (default): on overflow take the closest pair in normalised
%              objective space and drop the member farther from the ideal
%              point, never the best member of an objective; 'crowding':
%              drop the smallest crowding distance). Ignored when density='nn'.
%     mutation_prob (0; probability a particle gets polynomial mutation,
%              each dimension with prob 1/ndim)  mutation_eta (20)
%     dedupe (true) dedupe_tol (1e-12, relative to max(1,|f|))
%     pbest_rule ('dominate_or_random' or 'dominate' = strict dominance only)
%     compromise_method ('ideal_point'|'origin_maxnorm'|'knee')  compromise_weights ([])
%     hv_ref ([]: no hypervolume history)  hv_samples (2e4)  hv_seed (1)
%     init_X ([]: k x ndim rows placed in the initial swarm, e.g. baseline gains)
%     bound_tol (1e-6, fraction of the search range counted as on a bound)
%     restore_rng (true: restore the caller's RNG state on exit)
%     verbose (false)  print_every (5)
%     legacy (false): preset reproducing the original notebook behaviour:
%       density 'nn' on raw objectives (leader roulette and pruning), no
%       velocity clamp, 'clip' bounds, no dedupe, strict-dominance pbest,
%       linear search, no mutation, 'origin_maxnorm' compromise. Legacy
%       overrides those options. Non-finite objectives are rejected in all
%       modes, and an all-zero density uses a uniform leader instead of the
%       original crash (counted in info.guide_fallbacks).
%
%   arch_X, arch_F : final non-dominated archive (sorted by objectives).
%   best : struct (idx, x, f, score, method, weights) from mopso_select_compromise.
%   info : evals, X_all, F_all, iter_all, history (per iteration: archive_size,
%          hv, n_evals, n_accepted, n_dup_rejected, guide_fallbacks),
%          bound_frac / bound_frac_lb / bound_frac_ub (archive members on a
%          bound, per dimension), eval_bound_frac (all evaluations),
%          bound_margin (median normalised distance of archive members to the
%          nearest bound, search space), counters, effective opts.
%
%   RNG: a single stream, rng(seed,'twister') at the start; only rand() is used.
%   The MOPSO stream is saved before and restored after every objective call,
%   so an objective that uses or re-seeds the global RNG cannot perturb the
%   search (and vectorized/per-particle runs stay identical).

if nargin < 4 || isempty(opts)
    opts = struct();
end
lb = double(lb(:).');
ub = double(ub(:).');
ndim = numel(lb);
if numel(ub) ~= ndim
    error('mopso:bounds', 'lb and ub must have the same length');
end
if any(~isfinite(lb)) || any(~isfinite(ub)) || any(ub < lb)
    error('mopso:bounds', 'bounds must be finite with lb <= ub');
end
o = local_options(opts, ndim);
if any(o.log_scale & lb <= 0)
    error('mopso:log_scale', 'log_scale dimensions need lb > 0');
end

t_start = tic;
caller_rng = rng;
if o.restore_rng
    restore_guard = onCleanup(@() rng(caller_rng)); %#ok<NASGU>
end
rng(o.seed, 'twister');

% search space (z): log10 for log_scale dims
zlb = lb;
zub = ub;
zlb(o.log_scale) = log10(lb(o.log_scale));
zub(o.log_scale) = log10(ub(o.log_scale));
zrange = zub - zlb;
vmax = o.vel_max_frac .* zrange;
vmax(zrange == 0) = 0;

np = o.n_particles;
Z = zlb + rand(np, ndim) .* zrange;
if ~isempty(o.init_X)
    k = min(size(o.init_X, 1), np);
    Xi = min(max(o.init_X(1:k, :), lb), ub);
    Zi = Xi;
    Zi(:, o.log_scale) = log10(Xi(:, o.log_scale));
    Z(1:k, :) = Zi;
end
V = zeros(np, ndim);

pbest_Z = Z;
pbest_F = [];
AZ = zeros(0, ndim);
AF = [];
nobj = [];

n_total = np * o.n_iter;
X_all = zeros(n_total, ndim);
F_all = [];
iter_all = zeros(n_total, 1);
hist.iter = (1:o.n_iter).';
hist.archive_size = zeros(o.n_iter, 1);
hist.hv = NaN(o.n_iter, 1);
hist.n_evals = zeros(o.n_iter, 1);
hist.n_accepted = zeros(o.n_iter, 1);
hist.n_dup_rejected = zeros(o.n_iter, 1);
hist.guide_fallbacks = zeros(o.n_iter, 1);
n_eval = 0;
n_nonfinite = 0;
n_dup = 0;
n_fallback = 0;

for it = 1:o.n_iter
    X = local_to_x(Z, o.log_scale, lb, ub);
    F = local_evaluate(objfun, X, o.vectorized);
    if isempty(nobj)
        nobj = size(F, 2);
        F_all = zeros(n_total, nobj);
        pbest_F = F;
        if ~isempty(o.hv_ref) && numel(o.hv_ref) ~= nobj
            error('mopso:hv_ref', 'hv_ref must have %d elements', nobj);
        end
    elseif size(F, 2) ~= nobj
        error('mopso:objfun', 'objective count changed from %d to %d', nobj, size(F, 2));
    end
    rows = n_eval + (1:np);
    X_all(rows, :) = X;
    F_all(rows, :) = F;
    iter_all(rows) = it;
    n_eval = n_eval + np;

    % personal bests
    if it > 1
        for i = 1:np
            if local_pbest_replace(F(i, :), pbest_F(i, :), o.pbest_rule)
                pbest_F(i, :) = F(i, :);
                pbest_Z(i, :) = Z(i, :);
            end
        end
    end

    % archive, one candidate at a time in particle order
    n_acc = 0;
    n_dup_it = 0;
    for i = 1:np
        [AZ, AF, status] = local_archive_insert(AZ, AF, Z(i, :), F(i, :), o);
        if status == 1
            n_acc = n_acc + 1;
        elseif status == -1
            n_dup_it = n_dup_it + 1;
        elseif status == -2
            n_nonfinite = n_nonfinite + 1;
        end
    end
    n_dup = n_dup + n_dup_it;

    hist.archive_size(it) = size(AF, 1);
    hist.n_evals(it) = n_eval;
    hist.n_accepted(it) = n_acc;
    hist.n_dup_rejected(it) = n_dup_it;
    if ~isempty(o.hv_ref) && ~isempty(AF)
        hist.hv(it) = hypervolume_mc(AF, o.hv_ref, o.hv_samples, o.hv_seed);
    elseif ~isempty(o.hv_ref)
        hist.hv(it) = 0;
    end

    % move the swarm (not needed after the last evaluation)
    n_fb_it = 0;
    if it < o.n_iter && ~isempty(AF)
        dens = local_density(AF, o);
        if numel(o.w) == 2
            wk = o.w(1) + (o.w(2) - o.w(1)) * (it - 1) / max(1, o.n_iter - 2);
        else
            wk = o.w;
        end
        for i = 1:np
            [g, fb] = local_guide(dens, o.guide_selection);
            n_fb_it = n_fb_it + fb;
            r1 = rand(1, ndim);
            r2 = rand(1, ndim);
            v = wk * V(i, :) + o.c1 * r1 .* (pbest_Z(i, :) - Z(i, :)) ...
                + o.c2 * r2 .* (AZ(g, :) - Z(i, :));
            v = min(max(v, -vmax), vmax);
            z = Z(i, :) + v;
            [z, v] = local_bounds(z, v, zlb, zub, o.bound_handling);
            if o.mutation_prob > 0
                z = local_mutate(z, zlb, zub, o.mutation_prob, o.mutation_eta);
            end
            Z(i, :) = z;
            V(i, :) = v;
        end
    end
    if it < o.n_iter && isempty(AF)
        % nothing feasible yet (all objectives non-finite): re-sample the swarm
        Z = zlb + rand(np, ndim) .* zrange;
        V = zeros(np, ndim);
    end
    hist.guide_fallbacks(it) = n_fb_it;
    n_fallback = n_fallback + n_fb_it;

    if o.verbose && (mod(it, o.print_every) == 0 || it == 1 || it == o.n_iter)
        if isnan(hist.hv(it))
            fprintf('  MOPSO iter %3d/%d | archive %3d | evals %5d\n', ...
                it, o.n_iter, size(AF, 1), n_eval);
        else
            fprintf('  MOPSO iter %3d/%d | archive %3d | evals %5d | HV %.6g\n', ...
                it, o.n_iter, size(AF, 1), n_eval, hist.hv(it));
        end
    end
end

% output archive, sorted by objectives
if ~isempty(AF)
    [arch_F, ord] = sortrows(AF);
    AZ = AZ(ord, :);
else
    arch_F = zeros(0, nobj);
end
arch_X = local_to_x(AZ, o.log_scale, lb, ub);

best = struct('idx', [], 'x', [], 'f', [], 'score', [], ...
    'method', o.compromise_method, 'weights', o.compromise_weights);
if ~isempty(arch_F)
    [bi, sc] = mopso_select_compromise(arch_F, o.compromise_weights, o.compromise_method);
    best.idx = bi;
    best.x = arch_X(bi, :);
    best.f = arch_F(bi, :);
    best.score = sc(bi);
end

% bound-hitting report (search-space units)
tolz = o.bound_tol * max(zrange, realmin);
[f_lb, f_ub] = local_bound_frac(AZ, zlb, zub, tolz);
Zall = X_all;
Zall(:, o.log_scale) = log10(X_all(:, o.log_scale));
[e_lb, e_ub] = local_bound_frac(Zall, zlb, zub, tolz);

info.evals = n_eval;
info.X_all = X_all;
info.F_all = F_all;
info.iter_all = iter_all;
info.history = hist;
info.bound_frac_lb = f_lb;
info.bound_frac_ub = f_ub;
info.bound_frac = min(1, f_lb + f_ub);
info.eval_bound_frac = min(1, e_lb + e_ub);
if isempty(AZ)
    info.bound_margin = NaN(1, ndim);
else
    marg = min(AZ - zlb, zub - AZ) ./ max(zrange, realmin);
    info.bound_margin = median(marg, 1);
end
info.n_nonfinite = n_nonfinite;
info.n_dup_rejected = n_dup;
info.guide_fallbacks = n_fallback;
info.pbest_X = local_to_x(pbest_Z, o.log_scale, lb, ub);
info.pbest_F = pbest_F;
info.opts = o;
info.runtime_s = toc(t_start);

if o.verbose
    fprintf('  MOPSO done: %d evals, archive %d, %.2f s\n', n_eval, size(arch_F, 1), info.runtime_s);
    if any(info.bound_frac > 0.5)
        fprintf('  MOPSO warning: >50%% of archive on a bound in dim(s) %s\n', ...
            mat2str(find(info.bound_frac > 0.5)));
    end
end
end

% ---------------------------------------------------------------------------
function o = local_options(opts, ndim)
o.n_particles = local_get(opts, 'n_particles', 20);
o.n_iter = local_get(opts, 'n_iter', 30);
o.seed = local_get(opts, 'seed', 42);
o.archive_size = local_get(opts, 'archive_size', 60);
o.w = local_get(opts, 'w', 0.6);
o.c1 = local_get(opts, 'c1', 1.5);
o.c2 = local_get(opts, 'c2', 1.5);
o.log_scale = local_get(opts, 'log_scale', false);
o.vel_max_frac = local_get(opts, 'vel_max_frac', 0.2);
o.bound_handling = lower(local_get(opts, 'bound_handling', 'reset_velocity'));
o.normalize_objectives = local_get(opts, 'normalize_objectives', true);
o.density = lower(local_get(opts, 'density', 'crowding'));
o.dedupe = local_get(opts, 'dedupe', true);
o.dedupe_tol = local_get(opts, 'dedupe_tol', 1e-12);
o.pbest_rule = lower(local_get(opts, 'pbest_rule', 'dominate_or_random'));
o.compromise_method = lower(local_get(opts, 'compromise_method', 'ideal_point'));
o.compromise_weights = local_get(opts, 'compromise_weights', []);
o.hv_ref = local_get(opts, 'hv_ref', []);
o.hv_samples = local_get(opts, 'hv_samples', 2e4);
o.hv_seed = local_get(opts, 'hv_seed', 1);
o.init_X = local_get(opts, 'init_X', []);
o.bound_tol = local_get(opts, 'bound_tol', 1e-6);
o.restore_rng = local_get(opts, 'restore_rng', true);
o.vectorized = local_get(opts, 'vectorized', false);
o.verbose = local_get(opts, 'verbose', false);
o.print_every = local_get(opts, 'print_every', 5);
o.legacy = local_get(opts, 'legacy', false);
o.mutation_prob = local_get(opts, 'mutation_prob', 0);
o.guide_selection = lower(local_get(opts, 'guide_selection', 'roulette'));
o.prune = lower(local_get(opts, 'prune', 'pair'));
o.mutation_eta = local_get(opts, 'mutation_eta', 20);

if o.legacy
    o.normalize_objectives = false;
    o.density = 'nn';
    o.vel_max_frac = Inf;
    o.bound_handling = 'clip';
    o.dedupe = false;
    o.pbest_rule = 'dominate';
    o.compromise_method = 'origin_maxnorm';
    o.log_scale = false;
    o.mutation_prob = 0;
    o.guide_selection = 'roulette';
end

lgm = logical(o.log_scale(:).');
if isscalar(lgm)
    lgm = repmat(lgm, 1, ndim);
end
if numel(lgm) ~= ndim
    error('mopso:log_scale', 'log_scale must be scalar or 1 x %d', ndim);
end
o.log_scale = lgm;
vf = double(o.vel_max_frac(:).');
if isscalar(vf)
    vf = repmat(vf, 1, ndim);
end
o.vel_max_frac = vf;
if ~any(strcmp(o.bound_handling, {'reset_velocity', 'reflect', 'clip'}))
    error('mopso:opts', 'bound_handling must be reset_velocity, reflect or clip');
end
if ~any(strcmp(o.density, {'crowding', 'nn'}))
    error('mopso:opts', 'density must be crowding or nn');
end
if ~any(strcmp(o.guide_selection, {'roulette', 'tournament', 'uniform'}))
    error('mopso:opts', 'guide_selection must be roulette, tournament or uniform');
end
if ~any(strcmp(o.prune, {'pair', 'crowding'}))
    error('mopso:opts', 'prune must be pair or crowding');
end
if ~any(strcmp(o.pbest_rule, {'dominate_or_random', 'dominate'}))
    error('mopso:opts', 'pbest_rule must be dominate_or_random or dominate');
end
if numel(o.w) > 2 || o.n_particles < 1 || o.n_iter < 1 || o.archive_size < 1
    error('mopso:opts', 'invalid w, n_particles, n_iter or archive_size');
end
if ~isempty(o.init_X) && size(o.init_X, 2) ~= ndim
    error('mopso:opts', 'init_X must have %d columns', ndim);
end
end

function v = local_get(s, name, default)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end

function X = local_to_x(Z, logmask, lb, ub)
X = Z;
if any(logmask)
    X(:, logmask) = 10 .^ Z(:, logmask);
    X = min(max(X, lb), ub);
end
end

function F = local_evaluate(objfun, X, vectorized)
np = size(X, 1);
s = rng;
if vectorized
    F = objfun(X);
    if size(F, 1) ~= np
        error('mopso:objfun', 'vectorized objfun must return %d rows', np);
    end
else
    f1 = objfun(X(1, :));
    F = zeros(np, numel(f1));
    F(1, :) = f1(:).';
    for i = 2:np
        fi = objfun(X(i, :));
        if numel(fi) ~= size(F, 2)
            error('mopso:objfun', 'objfun returned %d objectives, expected %d', numel(fi), size(F, 2));
        end
        F(i, :) = fi(:).';
    end
end
rng(s);
F = double(F);
F(isnan(F)) = Inf;
end

function tf = local_dominates(a, b)
tf = all(a <= b) && any(a < b);
end

function tf = local_pbest_replace(f, pf, rule)
f_ok = all(isfinite(f));
p_ok = all(isfinite(pf));
if strcmp(rule, 'dominate')
    tf = local_dominates(f, pf);
    return;
end
% dominate_or_random: one rand() is drawn only for mutually non-dominated pairs
if ~f_ok && p_ok
    tf = false;
elseif f_ok && ~p_ok
    tf = true;
elseif local_dominates(f, pf)
    tf = true;
elseif local_dominates(pf, f)
    tf = false;
else
    tf = rand < 0.5;
end
end

function [AZ, AF, status] = local_archive_insert(AZ, AF, z, f, o)
% status: 1 accepted, 0 dominated, -1 duplicate, -2 non-finite
if any(~isfinite(f))
    status = -2;
    return;
end
if ~isempty(AF)
    if any(all(AF <= f, 2) & any(AF < f, 2))
        status = 0;
        return;
    end
    if o.dedupe
        tol = o.dedupe_tol * max(1, max(abs(f)));
        if any(all(abs(AF - f) <= tol, 2))
            status = -1;
            return;
        end
    end
    worse = all(f <= AF, 2) & any(f < AF, 2);
    AZ(worse, :) = [];
    AF(worse, :) = [];
end
AZ(end + 1, :) = z;
AF(end + 1, :) = f;
status = 1;
while size(AF, 1) > o.archive_size
    k = local_prune_index(AF, o);
    AZ(k, :) = [];
    AF(k, :) = [];
end
end

function dens = local_density(AF, o)
if strcmp(o.density, 'nn')
    dens = mopso_nn_distance(AF, o.normalize_objectives);
else
    dens = mopso_crowding(AF, o.normalize_objectives);
end
end

function k = local_prune_index(AF, o)
% index of the archive member to drop when the archive overflows
if strcmp(o.density, 'nn')
    [~, k] = min(local_density(AF, o));
    return;
end
if strcmp(o.prune, 'pair')
    k = local_prune_pair(AF, o.normalize_objectives);
    return;
end
% smallest crowding distance; ties (e.g. several Inf) broken by NN distance
dens = local_density(AF, o);
cand = find(dens == min(dens));
if numel(cand) > 1
    nn = mopso_nn_distance(AF, o.normalize_objectives);
    [~, j] = min(nn(cand));
    k = cand(j);
else
    k = cand;
end
end

function k = local_prune_pair(AF, normalize)
% closest pair in (normalised) objective space; drop the member farther from
% the ideal point. The best member of each objective is never dropped unless
% every member is protected.
if normalize
    Fn = mopso_minmax_norm(AF);
else
    Fn = AF - min(AF, [], 1);
end
n = size(Fn, 1);
D2 = zeros(n, n);
for j = 1:size(Fn, 2)
    c = Fn(:, j);
    D2 = D2 + (c - c.').^2;
end
D2(1:n+1:end) = Inf;
% protect the best member of each objective (defines the front's extent)
prot = false(n, 1);
for j = 1:size(AF, 2)
    [~, jm] = min(AF(:, j));
    prot(jm) = true;
end
D2(prot, prot) = Inf;
[dmin, jn] = min(D2, [], 2);
[~, a] = min(dmin);
b = jn(a);
if prot(a)
    k = b;
elseif prot(b)
    k = a;
elseif sum(Fn(a, :).^2) >= sum(Fn(b, :).^2)
    k = a;
else
    k = b;
end
end

function [z, v] = local_bounds(z, v, zlb, zub, mode)
lo = z < zlb;
hi = z > zub;
switch mode
    case 'clip'
        z = min(max(z, zlb), zub);
    case 'reset_velocity'
        z = min(max(z, zlb), zub);
        v(lo | hi) = 0;
    case 'reflect'
        z(lo) = 2 * zlb(lo) - z(lo);
        z(hi) = 2 * zub(hi) - z(hi);
        v(lo | hi) = -v(lo | hi);
        z = min(max(z, zlb), zub);
end
end

function [g, fb] = local_guide(dens, mode)
% leader choice; every branch draws exactly two rand() values
n = numel(dens);
switch mode
    case 'roulette'
        [g, fb] = mopso_select_guide(dens);
        rand;
    case 'tournament'
        r = rand(1, 2);
        a = min(n, floor(r(1) * n) + 1);
        b = min(n, floor(r(2) * n) + 1);
        da = dens(a);
        db = dens(b);
        if isnan(da)
            da = -Inf;
        end
        if isnan(db)
            db = -Inf;
        end
        if db > da
            g = b;
        else
            g = a;
        end
        fb = false;
    otherwise
        r = rand(1, 2);
        g = min(n, floor(r(1) * n) + 1);
        fb = false;
end
end

function z = local_mutate(z, zlb, zub, pm, eta)
% polynomial mutation (Deb) of a particle with probability pm; each dimension
% mutated with probability 1/ndim. Always draws 1 + 2*ndim rand() values.
nd = numel(z);
sel = rand < pm;
rd = rand(1, nd);
ru = rand(1, nd);
if ~sel
    return;
end
span = zub - zlb;
for j = 1:nd
    if rd(j) >= 1 / nd || span(j) <= 0
        continue;
    end
    d1 = (z(j) - zlb(j)) / span(j);
    d2 = (zub(j) - z(j)) / span(j);
    mp = 1 / (eta + 1);
    if ru(j) < 0.5
        val = 2 * ru(j) + (1 - 2 * ru(j)) * (1 - d1)^(eta + 1);
        dq = val^mp - 1;
    else
        val = 2 * (1 - ru(j)) + 2 * (ru(j) - 0.5) * (1 - d2)^(eta + 1);
        dq = 1 - val^mp;
    end
    z(j) = min(max(z(j) + dq * span(j), zlb(j)), zub(j));
end
end

function [f_lb, f_ub] = local_bound_frac(Z, zlb, zub, tolz)
n = size(Z, 1);
if n == 0
    f_lb = zeros(1, numel(zlb));
    f_ub = f_lb;
    return;
end
f_lb = sum(Z <= zlb + tolz, 1) / n;
f_ub = sum(Z >= zub - tolz, 1) / n;
both = sum((Z <= zlb + tolz) & (Z >= zub - tolz), 1) / n;
f_ub = f_ub - both;
end
