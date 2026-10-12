function results = test_mopso(seed_offset)
% TEST_MOPSO  Tests for mopso.m, mopso_select_compromise.m, hypervolume_mc.m.
%   results = test_mopso()  prints one PASS/FAIL line per check, returns a
%   struct array (name, passed, detail) and errors if any check failed.
%   test_mopso(k) shifts every random seed by k (robustness sweep; MATLAB and
%   Octave produce different twister streams for the same seed anyway).
%   Runtime: about 15 s in Octave 8.4.
%
%   GD thresholds (mean Euclidean distance of archive points to the true front):
%   ZDT1 (8 vars, 40 x 60 = 2400 evals): GD < 0.02, i.e. 2 % of the unit
%     objective range. The random initial swarm has GD ~ 3 (g ~ 5.5); over 38
%     seeds (1..8, 101..130) the corrected MOPSO reached 0.004-0.0095, so 0.02
%     keeps a 2x margin while still failing any run that has not converged.
%   DTLZ2 (3 obj, 7 vars, k = 5, 40 x 60): GD = mean | ||f|| - 1 | < 0.09.
%     Random points have E[g] = k/12 = 0.42; the same 38 seeds gave 0.018-0.059
%     (the legacy rules gave 0.062-0.086 on seeds 1..8). Pareto-archive PSO
%     converges slowly on 3+ objectives (dominance becomes rare), so 1.5x the
%     worst seed is used rather than an NSGA-II-style 1e-3 target.
%   HV may dip slightly when the bounded archive is pruned: allowed drop below
%   the running maximum is 1 % (ZDT1, worst seen 0.14 %) and 3 % (DTLZ2, worst
%   seen 1.5 %, Monte-Carlo HV). With an unbounded archive HV must be monotone.

if nargin < 1 || isempty(seed_offset)
    seed_offset = 0;
end
here = fileparts(mfilename('fullpath'));
addpath(fileparts(here));
t0 = tic;
R = struct('name', {}, 'passed', {}, 'detail', {});

R = run_check(R, 'hv_exact_known_cases', @t_hv_known, seed_offset);
R = run_check(R, 'hv_mc_vs_exact_2d', @t_hv_mc_vs_exact, seed_offset);
R = run_check(R, 'hv_preserves_caller_rng', @t_hv_rng, seed_offset);
R = run_check(R, 'guide_selection_edge_cases', @t_guide_edges, seed_offset);
R = run_check(R, 'density_all_duplicates', @t_density_dups, seed_offset);
R = run_check(R, 'compromise_methods', @t_compromise, seed_offset);
R = run_check(R, 'zdt1_front_quality', @t_zdt1, seed_offset);
R = run_check(R, 'zdt1_hv_monotone_unbounded_archive', @t_hv_monotone, seed_offset);
R = run_check(R, 'dtlz2_front_quality', @t_dtlz2, seed_offset);
R = run_check(R, 'seed_reproducibility', @t_seed, seed_offset);
R = run_check(R, 'vectorized_equals_per_particle', @t_vectorized, seed_offset);
R = run_check(R, 'objective_rng_isolation', @t_rng_isolation, seed_offset);
R = run_check(R, 'legacy_mode_runs', @t_legacy, seed_offset);
R = run_check(R, 'dedupe_quantised_objectives', @t_dedupe, seed_offset);
R = run_check(R, 'all_duplicate_archive_no_crash', @t_all_dup, seed_offset);
R = run_check(R, 'log_scale_finds_1e-6', @t_log_scale, seed_offset);
R = run_check(R, 'bound_stuck_report', @t_bounds, seed_offset);
R = run_check(R, 'options_and_errors', @t_options, seed_offset);

n_pass = sum([R.passed]);
fprintf('test_mopso: %d/%d checks passed (seed offset %d, %.1f s)\n', n_pass, numel(R), seed_offset, toc(t0));
results = R;
if n_pass < numel(R)
    error('test_mopso:failed', '%d check(s) failed', numel(R) - n_pass);
end
end

% ---------------------------------------------------------------------------
function R = run_check(R, name, fn, s)
try
    [ok, detail] = fn(s);
catch err
    ok = false;
    detail = ['ERROR: ' err.message];
end
k = numel(R) + 1;
R(k).name = name;
R(k).passed = logical(ok);
R(k).detail = detail;
if ok
    tag = 'PASS';
else
    tag = 'FAIL';
end
fprintf('[%s] %s: %s\n', tag, name, detail);
end

% ---------------------------------------------------------------------------
% test problems (vectorised: X is n x d)
function F = zdt1(X)
n = size(X, 2);
f1 = X(:, 1);
g = 1 + 9 * sum(X(:, 2:end), 2) / (n - 1);
F = [f1, g .* (1 - sqrt(f1 ./ g))];
end

function F = dtlz2(X)
M = 3;
g = sum((X(:, M:end) - 0.5).^2, 2);
a = X(:, 1) * pi / 2;
b = X(:, 2) * pi / 2;
F = [(1 + g) .* cos(a) .* cos(b), (1 + g) .* cos(a) .* sin(b), (1 + g) .* sin(a)];
end

function F = zdt1_noisy_rng(X)
% same as zdt1 but consumes and re-seeds the global RNG (must not matter)
F = zdt1(X);
rng(123);
dummy = rand(1, 5); %#ok<NASGU>
end

function F = zdt1_quantised(X)
F = round(zdt1(X) * 20) / 20;
end

function F = const_obj(X)
F = ones(size(X, 1), 2);
end

function F = one_d_four_obj(X)
x = X(:, 1);
F = [(x - 0.2).^2, (x - 1.5).^2, abs(x - 0.8), round(10 * x) / 10];
end

function F = log_problem(X)
e = (log10(X(:, 1)) + 6).^2;
F = [e + X(:, 2), e + 1 - X(:, 2)];
end

function F = bound_problem(X)
e = (X(:, 1) - 2).^2;
F = [e + X(:, 2), e + 1 - X(:, 2)];
end

function gd = gd_zdt1(F)
f1 = linspace(0, 1, 20001).';
T = [f1, 1 - sqrt(f1)];
d = zeros(size(F, 1), 1);
for i = 1:size(F, 1)
    d(i) = sqrt(min(sum((T - F(i, :)).^2, 2)));
end
gd = mean(d);
end

function gd = gd_dtlz2(F)
gd = mean(abs(sqrt(sum(F.^2, 2)) - 1));
end

function ok = no_duplicates(F)
ok = size(unique(F, 'rows'), 1) == size(F, 1);
end

function n = n_duplicates(F)
n = size(F, 1) - size(unique(F, 'rows'), 1);
end

function o = zdt1_opts(seed)
o = struct('n_particles', 40, 'n_iter', 60, 'archive_size', 60, 'seed', seed, ...
    'hv_ref', [1.1 1.1]);
end

% ---------------------------------------------------------------------------
function [ok, detail] = t_hv_known(s)
h1 = hypervolume_mc([0.5 0.5], [1 1]);
h2 = hypervolume_mc([0.2 0.8; 0.8 0.2], [1 1]);
h3 = hypervolume_mc([0.2 0.8; 0.8 0.2; 0.9 0.9; 1.2 0.1], [1 1]);
h4 = hypervolume_mc(0.25, 1);
[h5, se5] = hypervolume_mc([0.5 0.5 0.5], [1 1 1], 1e5, 3 + s);
% two 3-D boxes: inclusion-exclusion
P = [0.2 0.6 0.4; 0.6 0.2 0.5];
exact6 = prod(1 - P(1, :)) + prod(1 - P(2, :)) - prod(1 - max(P, [], 1));
[h6, se6] = hypervolume_mc(P, [1 1 1], 2e5, 4 + s);
h7 = hypervolume_mc([2 2], [1 1]);
ok = abs(h1 - 0.25) < 1e-15 && abs(h2 - 0.28) < 1e-15 && abs(h3 - 0.28) < 1e-15 ...
    && abs(h4 - 0.75) < 1e-15 && abs(h5 - 0.125) < 4 * se5 + 1e-3 ...
    && abs(h6 - exact6) < 4 * se6 + 1e-3 && h7 == 0;
detail = sprintf(['2-D exact %.4f/%.4f/%.4f, 1-D %.3f, 3-D MC box %.4f (0.125), ' ...
    'two boxes %.4f vs %.4f (se %.1e), outside ref %g'], h1, h2, h3, h4, h5, h6, exact6, se6, h7);
end

function [ok, detail] = t_hv_mc_vs_exact(s)
rng(11 + s, 'twister');
f1 = sort(rand(40, 1));
F = [f1, 1 - sqrt(f1) + 0.05 * rand(40, 1)];
F = [F; rand(20, 2) + 0.5];
ref = [1.1 1.1];
he = hypervolume_mc(F, ref);
[hm, se] = hypervolume_mc(F, ref, 2e5, 5 + s, 'mc');
err = abs(hm - he);
ok = err < max(4 * se, 0.005 * he);
detail = sprintf('exact %.5f, MC %.5f, |err| %.2e, 4*se %.2e', he, hm, err, 4 * se);
end

function [ok, detail] = t_hv_rng(s)
rng(99, 'twister');
a = rand(1, 3);
rng(99, 'twister');
hypervolume_mc([0.2 0.3 0.4; 0.5 0.1 0.6], [1 1 1], 5e4, 7);
b = rand(1, 3);
ok = isequal(a, b);
detail = sprintf('caller stream unchanged after MC call: %d', ok);
end

function [ok, detail] = t_guide_edges(s)
rng(1 + s, 'twister');
[i1, fb1] = mopso_select_guide(zeros(5, 1));
[i2, fb2] = mopso_select_guide(Inf(4, 1));
[i3, fb3] = mopso_select_guide([NaN; NaN; NaN]);
[i4, fb4] = mopso_select_guide([0; Inf; 0; Inf]);
[i5, fb5] = mopso_select_guide(7);
picks = zeros(300, 1);
for k = 1:300
    picks(k) = mopso_select_guide([0; 2; 0]);
end
picks_inf = zeros(300, 1);
for k = 1:300
    picks_inf(k) = mopso_select_guide([Inf; 1; 1; Inf]);
end
share_end = mean(picks_inf == 1 | picks_inf == 4);
ok = any(i1 == 1:5) && fb1 && any(i2 == 1:4) && fb2 && any(i3 == 1:3) && fb3 ...
    && any(i4 == [2 4]) && fb4 && i5 == 1 && ~fb5 && all(picks == 2) ...
    && share_end > 0.35 && share_end < 0.65;
detail = sprintf(['all-zero->%d, all-Inf->%d, NaN->%d, zero/Inf->%d (fallbacks flagged), ' ...
    'zero weights never picked, Inf share %.2f (expect 0.5)'], i1, i2, i3, i4, share_end);
end

function [ok, detail] = t_density_dups(s)
F = repmat([1 2 3], 6, 1);
c = mopso_crowding(F, true);
d = mopso_nn_distance(F, false);
dn = mopso_nn_distance(F, true);
c2 = mopso_crowding([0 1; 0.5 0.5; 1 0], true);
c3 = mopso_crowding([0 1; 0.5 0.5; 1 0], false);
raw = mopso_nn_distance([0 0; 1 100; 2 0], false);
nrm = mopso_nn_distance([0 0; 1 100; 2 0], true);
ok = all(c == 0) && all(d == 0) && all(dn == 0) && isequal(isinf(c2), [true; false; true]) ...
    && abs(c2(2) - 2) < 1e-12 && abs(c3(2) - 2) < 1e-12 ...
    && raw(1) == 2 && abs(nrm(1) - 1) < 1e-12;
detail = sprintf(['all-duplicate crowding/NN = 0, interior crowding %.2f, ' ...
    'raw NN %.1f vs normalised %.2f (unit bias removed)'], c2(2), raw(1), nrm(1));
end

function [ok, detail] = t_compromise(s)
% large offset in f1: the origin rule ignores f1 and picks an extreme
F = [100 1; 100.5 0.4; 101 0];
i_ideal = mopso_select_compromise(F, [], 'ideal_point');
i_orig = mopso_select_compromise(F, [], 'origin_maxnorm');
% weighted: favouring f2 moves the ideal-point choice to the f2 extreme
i_w = mopso_select_compromise(F, [1 10], 'ideal_point');
% constant column must not produce NaN
[i_c, s_c] = mopso_select_compromise([1 5; 2 5; 3 5], [], 'ideal_point');
% knee
K = [0 1; 0.05 0.2; 0.4 0.1; 1 0];
[i_k, s_k] = mopso_select_compromise(K, [], 'knee');
% knee with >2 objectives falls back to ideal point
F3 = [0 1 1; 0.5 0.5 0.5; 1 0 0];
i_k3 = mopso_select_compromise(F3, [], 'knee');
i_i3 = mopso_select_compromise(F3, [], 'ideal_point');
% non-finite row never chosen
i_nf = mopso_select_compromise([NaN 0; 0.5 0.5; 1 1], [], 'ideal_point');
% notebook equivalence: equal weights reproduce argmin ||F./max(F)||
rng(5 + s, 'twister');
G = rand(30, 4);
nb = G ./ (max(G, [], 1) + 1e-12);
[~, i_nb] = min(sqrt(sum(nb.^2, 2)));
i_om = mopso_select_compromise(G, [1 1 1 1], 'origin_maxnorm');
ok = i_ideal == 2 && i_orig == 3 && i_w == 3 && i_c == 1 && all(isfinite(s_c)) ...
    && i_k == 2 && s_k(2) < 0 && i_k3 == i_i3 && i_nf == 2 && i_om == i_nb;
detail = sprintf(['offset front: ideal_point->%d, origin_maxnorm->%d (extreme), ' ...
    'weights [1 10]->%d, constant col->%d, knee->%d, notebook rule match %d'], ...
    i_ideal, i_orig, i_w, i_c, i_k, i_om == i_nb);
end

function [ok, detail] = t_zdt1(s)
o = zdt1_opts(1 + s);
[AX, AF, best, info] = mopso(@zdt1, zeros(1, 8), ones(1, 8), o);
gd = gd_zdt1(AF);
gd0 = gd_zdt1(info.F_all(info.iter_all == 1, :));
hv = info.history.hv;
q = round(numel(hv) / 4);
hv_up = hv(end) > hv(2 * q) && hv(2 * q) > hv(q) && hv(q) >= hv(1);
max_drop = max(0, max(cummax(hv) - hv));
inb = all(info.X_all(:) >= 0) && all(info.X_all(:) <= 1);
nd = all(mopso_nondominated(AF));
ok = nd && no_duplicates(AF) && size(AF, 1) <= o.archive_size && gd < 0.02 ...
    && hv_up && max_drop < 0.01 * hv(end) && inb && info.evals == 2400 ...
    && isequal(size(info.X_all), [2400 8]) && isequal(size(info.bound_frac), [1 8]) ...
    && all(info.bound_frac >= 0 & info.bound_frac <= 1) && numel(best.x) == 8 ...
    && isequal(best.f, AF(best.idx, :));
detail = sprintf(['archive %d non-dominated %d, GD %.4f (init %.2f, thr 0.02), HV %.4f->%.4f->%.4f ' ...
    '(true 0.8767, max drop %.1e), bound frac x2..x8 mean %.2f'], size(AF, 1), nd, gd, gd0, ...
    hv(1), hv(2 * q), hv(end), max_drop, mean(info.bound_frac(2:end)));
end

function [ok, detail] = t_hv_monotone(s)
% with no pruning the archive only loses dominated points: HV cannot decrease
o = struct('n_particles', 20, 'n_iter', 30, 'archive_size', 1e5, 'seed', 3 + s, 'hv_ref', [1.1 1.1]);
[~, AF, ~, info] = mopso(@zdt1, zeros(1, 6), ones(1, 6), o);
d = diff(info.history.hv);
ok = all(d >= -1e-12) && info.history.hv(end) > info.history.hv(1) && all(mopso_nondominated(AF));
detail = sprintf('archive %d, min HV step %.2e, HV %.4f -> %.4f', size(AF, 1), min(d), ...
    info.history.hv(1), info.history.hv(end));
end

function [ok, detail] = t_dtlz2(s)
o = struct('n_particles', 40, 'n_iter', 60, 'archive_size', 80, 'seed', 1 + s, ...
    'hv_ref', [1.1 1.1 1.1], 'hv_samples', 2e4);
[~, AF, best, info] = mopso(@dtlz2, zeros(1, 7), ones(1, 7), o);
gd = gd_dtlz2(AF);
gd0 = gd_dtlz2(info.F_all(info.iter_all == 1, :));
hv = info.history.hv;
q = round(numel(hv) / 4);
max_drop = max(0, max(cummax(hv) - hv));
hv_true = 1.1^3 - pi / 6;
nd = all(mopso_nondominated(AF));
ok = nd && no_duplicates(AF) && size(AF, 1) <= 80 && gd < 0.09 && gd < 0.25 * gd0 ...
    && hv(end) > hv(2 * q) && hv(2 * q) > hv(1) && max_drop < 0.03 * hv(end) ...
    && numel(best.f) == 3;
detail = sprintf(['archive %d, GD %.4f (init %.3f, thr 0.09), HV %.3f->%.3f->%.3f of %.3f ' ...
    '(max drop %.1e)'], size(AF, 1), gd, gd0, hv(1), hv(2 * q), hv(end), hv_true, max_drop);
end

function [ok, detail] = t_seed(s)
o = struct('n_particles', 20, 'n_iter', 25, 'seed', 7 + s);
[X1, F1] = mopso(@zdt1, zeros(1, 6), ones(1, 6), o);
[X2, F2] = mopso(@zdt1, zeros(1, 6), ones(1, 6), o);
o.hv_ref = [1.1 1.1];
o.hv_samples = 1000;
[X3, F3] = mopso(@zdt1, zeros(1, 6), ones(1, 6), o);
o = rmfield(o, 'hv_ref');
o.seed = 8 + s;
[~, F4] = mopso(@zdt1, zeros(1, 6), ones(1, 6), o);
% caller RNG state restored on exit
rng(2024, 'twister');
a = rand(1, 4);
rng(2024, 'twister');
mopso(@zdt1, zeros(1, 6), ones(1, 6), struct('n_particles', 5, 'n_iter', 3));
b = rand(1, 4);
same = isequal(F1, F2) && isequal(X1, X2);
same_hv = isequal(F1, F3) && isequal(X1, X3);
differ = ~isequal(size(F1), size(F4)) || ~isequal(F1, F4);
ok = same && same_hv && differ && isequal(a, b);
detail = sprintf(['same seed identical %d, hv history does not perturb stream %d, ' ...
    'different seed differs %d, caller RNG restored %d'], same, same_hv, differ, isequal(a, b));
end

function [ok, detail] = t_vectorized(s)
o = struct('n_particles', 20, 'n_iter', 25, 'seed', 5 + s);
[X1, F1, b1, i1] = mopso(@zdt1, zeros(1, 6), ones(1, 6), o);
o.vectorized = true;
[X2, F2, b2, i2] = mopso(@zdt1, zeros(1, 6), ones(1, 6), o);
o.legacy = true;
[X3, F3] = mopso(@zdt1, zeros(1, 6), ones(1, 6), o);
o.vectorized = false;
[X4, F4] = mopso(@zdt1, zeros(1, 6), ones(1, 6), o);
ok = isequal(F1, F2) && isequal(X1, X2) && isequal(i1.X_all, i2.X_all) ...
    && isequal(b1.idx, b2.idx) && isequal(F3, F4) && isequal(X3, X4);
detail = sprintf('archive %d rows identical, all %d evaluations identical, legacy identical %d', ...
    size(F1, 1), i1.evals, isequal(F3, F4));
end

function [ok, detail] = t_rng_isolation(s)
o = struct('n_particles', 20, 'n_iter', 20, 'seed', 9 + s);
[X1, F1] = mopso(@zdt1, zeros(1, 6), ones(1, 6), o);
[X2, F2] = mopso(@zdt1_noisy_rng, zeros(1, 6), ones(1, 6), o);
o.vectorized = true;
[X3, F3] = mopso(@zdt1_noisy_rng, zeros(1, 6), ones(1, 6), o);
ok = isequal(F1, F2) && isequal(X1, X2) && isequal(F1, F3) && isequal(X1, X3);
detail = sprintf('objective that re-seeds/consumes the RNG leaves the search unchanged: %d', ok);
end

function [ok, detail] = t_legacy(s)
o = zdt1_opts(1 + s);
o.legacy = true;
[AX, AF, best, info] = mopso(@zdt1, zeros(1, 8), ones(1, 8), o);
eo = info.opts;
preset = strcmp(eo.density, 'nn') && ~eo.normalize_objectives && all(isinf(eo.vel_max_frac)) ...
    && strcmp(eo.bound_handling, 'clip') && ~eo.dedupe && strcmp(eo.pbest_rule, 'dominate') ...
    && strcmp(eo.compromise_method, 'origin_maxnorm') && ~any(eo.log_scale) && eo.mutation_prob == 0;
gd = gd_zdt1(AF);
ok = preset && all(mopso_nondominated(AF)) && size(AF, 1) <= 60 && info.evals == 2400 ...
    && strcmp(best.method, 'origin_maxnorm') && gd < 0.05;
detail = sprintf(['legacy preset applied %d, archive %d, GD %.4f, bound frac x2..x8 mean %.2f ' ...
    '(ZDT1 optimum lies on x=0, which clip-with-momentum favours)'], preset, size(AF, 1), gd, ...
    mean(info.bound_frac(2:end)));
end

function [ok, detail] = t_dedupe(s)
o = struct('n_particles', 30, 'n_iter', 40, 'seed', 2 + s, 'archive_size', 60);
[~, F_new, ~, i_new] = mopso(@zdt1_quantised, zeros(1, 6), ones(1, 6), o);
o.legacy = true;
[~, F_old, ~, i_old] = mopso(@zdt1_quantised, zeros(1, 6), ones(1, 6), o);
o.legacy = false;
o.dedupe = false;
[~, F_nod] = mopso(@zdt1_quantised, zeros(1, 6), ones(1, 6), o);
ok = n_duplicates(F_new) == 0 && i_new.n_dup_rejected > 0 && n_duplicates(F_old) > 0 ...
    && n_duplicates(F_nod) > 0 && i_old.n_dup_rejected == 0;
detail = sprintf(['quantised objectives: corrected archive %d rows / 0 duplicates (%d rejected); ' ...
    'legacy %d rows / %d duplicates; dedupe=false %d duplicates'], size(F_new, 1), ...
    i_new.n_dup_rejected, size(F_old, 1), n_duplicates(F_old), n_duplicates(F_nod));
end

function [ok, detail] = t_all_dup(s)
% the original select_guide raised an error once the archive held only duplicates
o = struct('n_particles', 10, 'n_iter', 6, 'seed', 1 + s, 'archive_size', 8, 'legacy', true);
[AX, AF, best, info] = mopso(@const_obj, zeros(1, 3), ones(1, 3), o);
ok1 = size(AF, 1) == 8 && n_duplicates(AF) == 7 && info.guide_fallbacks > 0 && ~isempty(best.idx);
o.legacy = false;
[AX2, AF2, best2, info2] = mopso(@const_obj, zeros(1, 3), ones(1, 3), o);
ok2 = size(AF2, 1) == 1 && info2.n_dup_rejected == 59 && best2.idx == 1;
o.dedupe = false;
o.density = 'crowding';
[~, AF3, ~, info3] = mopso(@const_obj, zeros(1, 3), ones(1, 3), o);
ok3 = size(AF3, 1) == 8 && info3.guide_fallbacks > 0;
o.guide_selection = 'tournament';
[~, AF4] = mopso(@const_obj, zeros(1, 3), ones(1, 3), o);
% direct call on an all-duplicate archive
g = mopso_select_guide(mopso_nn_distance(ones(5, 2), false));
ok = ok1 && ok2 && ok3 && size(AF4, 1) == 8 && any(g == 1:5);
detail = sprintf(['legacy: archive %d all duplicates, %d uniform fallbacks (original crashed); ' ...
    'corrected: archive %d, %d duplicates rejected; dedupe off + crowding: %d fallbacks'], ...
    size(AF, 1), info.guide_fallbacks, size(AF2, 1), info2.n_dup_rejected, info3.guide_fallbacks);
end

function [ok, detail] = t_log_scale(s)
lb = [1e-8 0];
ub = [1e-3 1];
o = struct('n_particles', 20, 'n_iter', 30, 'seed', 1 + s, 'log_scale', [true false]);
[AX, AF, best, info] = mopso(@log_problem, lb, ub, o);
err = abs(log10(AX(:, 1)) + 6);
x_init = info.X_all(info.iter_all == 1, 1);
frac_low = mean(x_init < 1e-5);
inb = all(info.X_all(:, 1) >= lb(1)) && all(info.X_all(:, 1) <= ub(1)) ...
    && all(AX(:, 1) >= lb(1)) && all(AX(:, 1) <= ub(1));
o.log_scale = false;
[AXl, ~, bestl, infol] = mopso(@log_problem, lb, ub, o);
errl = abs(log10(AXl(:, 1)) + 6);
frac_low_lin = mean(infol.X_all(infol.iter_all == 1, 1) < 1e-5);
% thresholds from 120 seeds: median <= 0.034, best <= 0.023, >= 90 percent within 0.1
% decade, initial share below 1e-5 >= 0.35 (log-uniform expectation 0.6, linear 0.01)
ok = median(err) < 0.05 && mean(err < 0.1) >= 0.8 && abs(log10(best.x(1)) + 6) < 0.05 ...
    && frac_low > 0.2 && inb && all(info.opts.log_scale == [true false]);
detail = sprintf(['log search: median |log10(x1)+6| %.4f, %.0f%% within 0.1 decade, best x1 %.3g, ' ...
    '%.0f%% of initial swarm below 1e-5 | linear search (info only): median err %.4f, ' ...
    'best %.3g, %.0f%% below 1e-5'], median(err), 100 * mean(err < 0.1), best.x(1), ...
    100 * frac_low, median(errl), bestl.x(1), 100 * frac_low_lin);
end

function [ok, detail] = t_bounds(s)
% optimum of x1 lies outside [0,1] (at 2): the archive must report x1 on its upper bound
o = struct('n_particles', 20, 'n_iter', 30, 'seed', 1 + s);
[AX, ~, ~, info] = mopso(@bound_problem, [0 0], [1 1], o);
o.bound_handling = 'reflect';
[AXr, ~, ~, infor] = mopso(@bound_problem, [0 0], [1 1], o);
o.bound_handling = 'clip';
o.vel_max_frac = Inf;
[~, ~, ~, infoc] = mopso(@bound_problem, [0 0], [1 1], o);
inb = all(infor.X_all(:) >= 0 & infor.X_all(:) <= 1) && all(infoc.X_all(:) >= 0 & infoc.X_all(:) <= 1);
ok = info.bound_frac_ub(1) > 0.3 && info.bound_frac_lb(1) == 0 && info.bound_margin(1) < 0.01 ...
    && info.bound_margin(2) > 0.1 && infor.bound_margin(1) < 0.01 && inb ...
    && isequal(size(info.bound_frac), [1 2]) && isequal(size(info.eval_bound_frac), [1 2]) ...
    && all(AX(:, 1) <= 1) && all(AXr(:, 1) <= 1);
detail = sprintf(['reset_velocity: on-bound frac %s, margin %s; reflect: on-bound frac %s, ' ...
    'margin %s; clip: on-bound frac %s'], mat2str(info.bound_frac, 2), mat2str(info.bound_margin, 2), ...
    mat2str(infor.bound_frac, 2), mat2str(infor.bound_margin, 2), mat2str(infoc.bound_frac, 2));
end

function [ok, detail] = t_options(s)
lb = zeros(1, 4);
ub = ones(1, 4);
base = [0.25 0.5 0.75 0.1];
o = struct('n_particles', 10, 'n_iter', 5, 'init_X', base, 'w', [0.9 0.4], ...
    'mutation_prob', 0.5, 'guide_selection', 'tournament', 'prune', 'crowding', ...
    'archive_size', 5, 'compromise_method', 'knee', 'verbose', false, 'seed', 42 + s);
[~, AF, best, info] = mopso(@zdt1, lb, ub, o);
ok_init = isequal(info.X_all(1, :), base);
ok_run = size(AF, 1) <= 5 && all(mopso_nondominated(AF)) && ~isempty(best.idx);
% non-finite objectives are rejected and counted
[~, AFn, ~, infon] = mopso(@(X) zdt1(X) + 0 ./ (X(:, 1) > 0.5), lb, ub, ...
    struct('n_particles', 10, 'n_iter', 5, 'vectorized', true, 'seed', 42 + s));
ok_nf = infon.n_nonfinite > 0 && all(isfinite(AFn(:)));
n_err = 0;
try
    mopso(@(X) zeros(size(X, 1) + 1, 2), lb, ub, struct('vectorized', true, 'n_iter', 2));
catch
    n_err = n_err + 1;
end
try
    mopso(@zdt1, [0 -1], [1 1], struct('log_scale', [false true]));
catch
    n_err = n_err + 1;
end
try
    mopso(@zdt1, lb, ub, struct('bound_handling', 'bogus'));
catch
    n_err = n_err + 1;
end
% 1 decision variable, 4 objectives (shape of the FCS-MPC lambda_sw tuning)
[X1, F1, b1] = mopso(@one_d_four_obj, 0.01, 2, struct('n_particles', 10, 'n_iter', 10, 'seed', 3 + s));
ok_1d = size(X1, 2) == 1 && size(F1, 2) == 4 && all(mopso_nondominated(F1)) ...
    && no_duplicates(F1) && numel(b1.f) == 4;
ok = ok_init && ok_run && ok_nf && ok_1d && n_err == 3;
detail = sprintf(['init_X evaluated first %d, w schedule/mutation/tournament/crowding/knee run %d, ' ...
    'non-finite rejected %d (n=%d), 1-var/4-obj archive %d, bad inputs raise errors %d/3'], ...
    ok_init, ok_run, ok_nf, infon.n_nonfinite, size(F1, 1), n_err);
end
