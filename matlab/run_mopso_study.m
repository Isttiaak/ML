function res = run_mopso_study(cfg)
%RUN_MOPSO_STUDY  Multi-seed MOPSO tuning of every controller (corrected study).
%   res = run_mopso_study()      study_config() defaults
%   res = run_mopso_study(cfg)   cfg from study_config (missing fields filled)
%
%   For each controller in cfg.controllers and each seed in cfg.seeds, mopso
%   is run on objective_boost (vectorized: one batch of n_particles
%   simulations per iteration).  The notebook baseline gains (+ P.KPV/P.KIV
%   when the outer loop is tuned) are placed in the initial swarm
%   (cfg.init_baseline), so the search can never end worse than the baseline.
%
%   Hypervolume (HV): ONE normalisation per controller, shared by all seeds:
%   ideal = min and nadir = max over the non-penalised members of all seeds'
%   final archives, Fn = (F - ideal) ./ (nadir - ideal), reference point
%   cfg.hv.ref (1.1) in every objective.  Penalised members (never settled,
%   never recovered, diverged) lie beyond the reference and add nothing.
%   hv(seed) uses the final (bounded) archive; hv_hist(it, seed) uses the
%   non-dominated set of all evaluations up to iteration it (unbounded,
%   hence monotone and >= the archive HV).
%
%   Compromise: ideal-point rule (mopso_select_compromise) on the
%   non-penalised members.  Per seed on its archive, and the FINAL design on
%   the merged front (non-dominated union of all seeds' archives).
%   Bound report (search space, log10 where log_scale): margin = distance to
%   the nearest bound / range (0 = on the bound); 'on bound' if margin <
%   cfg.bound_tol_frac.  mopso's archive bound_frac / bound_margin are kept.
%
%   res fields:
%     cfg, controllers, runtime_s, gains_table.<ctrl> (gains, KPV, KIV of the
%     final compromise; use with run_robustness), baseline_table.<ctrl>,
%     summary (struct array), and ctrl.<name> with: S (search space),
%     obj_names, runs (per seed: seed, arch_X, arch_F, penalised, mopso_best,
%     comp_idx, comp_x, comp_f, comp_margin, hv, hv_hist, n_evals,
%     runtime_s, bound_frac, bound_margin, eval_bound_frac, n_nonfinite,
%     X_all, F_all, iter_all), norm (ideal, nadir, span, ref), hv (1 x
%     nseeds), hv_mean, hv_std, hv_hist (n_iter x nseeds), hv_hist_evals,
%     merged (X, F, penalised, comp_idx, x, f, gains, KPV, KIV, margin,
%     on_bound, bound_frac, hv), n_evals, runtime_s.
%   If cfg.out_file is not empty the struct is saved there (-v7 MAT file).

if nargin < 1
    cfg = [];
end
cfg = study_config(cfg);
P = cfg.P;
t_all = tic;
res.cfg = cfg;
res.controllers = cfg.controllers;
res.started = datestr(now, 31);
seeds = cfg.seeds(:).';
summary = cell(1, numel(cfg.controllers));

for c = 1:numel(cfg.controllers)
    name = cfg.controllers{c};
    S = drv_search_space(name, cfg);
    obj_names = drv_objective_names(name, cfg);
    o = drv_merge(cfg.mopso, getf(cfg.mopso_ctrl, name, struct()));
    o.log_scale = S.log_scale;
    o.vectorized = true;
    o.hv_ref = [];
    if cfg.init_baseline
        o.init_X = S.x0;
    else
        o.init_X = [];
    end
    objfun = @(X) objective_boost(X, name, P, cfg);
    if cfg.verbose
        fprintf('[%s] %d vars (%s), %d objectives, %d seeds x %d x %d evals, backend %s\n', ...
            name, numel(S.lb), strjoin(S.names, ' '), numel(obj_names), numel(seeds), ...
            getf(o, 'n_particles', 20), getf(o, 'n_iter', 30), cfg.backend);
    end
    t_c = tic;
    runs = cell(1, numel(seeds));
    for k = 1:numel(seeds)
        o.seed = seeds(k);
        t_r = tic;
        [AX, AF, best, info] = mopso(objfun, S.lb, S.ub, o);
        r = struct();
        r.seed = seeds(k);
        r.arch_X = AX;
        r.arch_F = AF;
        r.penalised = drv_penalised(AF, cfg);
        r.mopso_best = best;
        r.n_evals = info.evals;
        r.runtime_s = toc(t_r);
        r.bound_frac = info.bound_frac;
        r.bound_margin = info.bound_margin;
        r.eval_bound_frac = info.eval_bound_frac;
        r.n_nonfinite = info.n_nonfinite;
        r.X_all = info.X_all;
        r.F_all = info.F_all;
        r.iter_all = info.iter_all;
        runs{k} = r;
        if cfg.verbose
            fprintf('  seed %d: %d evals, archive %d (%d penalised), %.1f s\n', seeds(k), ...
                info.evals, size(AF, 1), sum(r.penalised), r.runtime_s);
        end
    end

    % common normalisation over all seeds of this controller
    allF = zeros(0, numel(obj_names));
    allpen = false(0, 1);
    for k = 1:numel(runs)
        allF = [allF; runs{k}.arch_F]; %#ok<AGROW>
        allpen = [allpen; runs{k}.penalised]; %#ok<AGROW>
    end
    base = allF(~allpen, :);
    if isempty(base)
        base = allF;
    end
    nrm.ideal = min(base, [], 1);
    nrm.nadir = max(base, [], 1);
    nrm.span = nrm.nadir - nrm.ideal;
    nrm.span(~(nrm.span > 0)) = 1;
    nrm.ref = cfg.hv.ref * ones(1, numel(obj_names));
    nrm.based_on_penalised = all(allpen);
    normf = @(F) (F - nrm.ideal) ./ nrm.span;

    n_iter = getf(o, 'n_iter', 30);
    hv = zeros(1, numel(runs));
    hv_hist = NaN(n_iter, numel(runs));
    for k = 1:numel(runs)
        r = runs{k};
        hv(k) = hypervolume_mc(normf(r.arch_F), nrm.ref, cfg.hv.samples, cfg.hv.seed);
        r.hv = hv(k);
        if cfg.hv.history
            nd = zeros(0, numel(obj_names));
            for it = 1:max(r.iter_all)
                Fi = [nd; r.F_all(r.iter_all == it, :)];
                Fi = Fi(all(isfinite(Fi), 2), :);
                nd = Fi(mopso_nondominated(Fi), :);
                hv_hist(it, k) = hypervolume_mc(normf(nd), nrm.ref, cfg.hv.hist_samples, cfg.hv.seed);
            end
        end
        r.hv_hist = hv_hist(:, k);
        [r.comp_idx, r.comp_x, r.comp_f] = compromise(r.arch_X, r.arch_F, r.penalised, o);
        r.comp_margin = drv_bound_margin(r.comp_x, S);
        runs{k} = r;
    end
    runs = [runs{:}];

    % merged front: non-dominated union of the archives
    MX = vertcat(runs.arch_X);
    MF = vertcat(runs.arch_F);
    [MF, iu] = unique(MF, 'rows');
    MX = MX(iu, :);
    nd = mopso_nondominated(MF);
    mg.X = MX(nd, :);
    mg.F = MF(nd, :);
    mg.penalised = drv_penalised(mg.F, cfg);
    [mg.comp_idx, mg.x, mg.f] = compromise(mg.X, mg.F, mg.penalised, o);
    [G, KPV, KIV] = drv_decode(mg.x, name, cfg);
    mg.gains = G;
    mg.KPV = KPV;
    mg.KIV = KIV;
    mg.margin = drv_bound_margin(mg.x, S);
    mg.on_bound = mg.margin < cfg.bound_tol_frac;
    mg.bound_frac = mean(drv_bound_margin(mg.X, S) < cfg.bound_tol_frac, 1);
    mg.hv = hypervolume_mc(normf(mg.F), nrm.ref, cfg.hv.samples, cfg.hv.seed);

    R = struct();
    R.S = S;
    R.obj_names = obj_names;
    R.mopso_opts = o;
    R.runs = runs;
    R.norm = nrm;
    R.hv = hv;
    R.hv_mean = mean(hv);
    if numel(hv) > 1
        R.hv_std = std(hv);
    else
        R.hv_std = 0;
    end
    R.hv_hist = hv_hist;
    R.hv_hist_evals = (1:n_iter)' * getf(o, 'n_particles', 20);
    R.merged = mg;
    R.n_evals = sum([runs.n_evals]);
    R.runtime_s = toc(t_c);
    res.ctrl.(name) = R;
    res.gains_table.(name) = struct('gains', G, 'KPV', KPV, 'KIV', KIV);
    res.baseline_table.(name) = struct('gains', P.gains_baseline.(name), 'KPV', P.KPV, 'KIV', P.KIV);

    cm = vertcat(runs.comp_margin);
    sm.ctrl = name;
    sm.n_vars = numel(S.lb);
    sm.n_evals = R.n_evals;
    sm.runtime_s = R.runtime_s;
    sm.hv_mean = R.hv_mean;
    sm.hv_std = R.hv_std;
    sm.front_size = size(mg.F, 1);
    sm.front_penalised = sum(mg.penalised);
    sm.comp_x = mg.x;
    sm.comp_f = mg.f;
    sm.comp_margin = mg.margin;
    sm.on_bound_dims = S.names(mg.on_bound);
    sm.seed_comp_on_bound_frac = mean(cm < cfg.bound_tol_frac, 1);
    summary{c} = sm;
end
res.summary = [summary{:}];
res.runtime_s = toc(t_all);
if cfg.verbose
    print_summary(res);
end
if ~isempty(cfg.out_file)
    save(cfg.out_file, 'res', '-v7');
    if cfg.verbose
        fprintf('saved %s\n', cfg.out_file);
    end
end
end

% ---------------------------------------------------------------------------
function [idx, x, f] = compromise(X, F, pen, o)
valid = find(~pen);
if isempty(valid)
    valid = (1:size(F, 1)).';
end
w = getf(o, 'compromise_weights', []);
method = getf(o, 'compromise_method', 'ideal_point');
if strcmp(method, 'origin_maxnorm') || getf(o, 'legacy', false)
    method = 'origin_maxnorm';
end
j = mopso_select_compromise(F(valid, :), w, method);
idx = valid(j);
x = X(idx, :);
f = F(idx, :);
end

function print_summary(res)
fprintf('\n%-12s %4s %6s %8s %17s %6s  %s\n', 'controller', 'vars', 'evals', 'time[s]', ...
    'HV mean +/- std', 'front', 'compromise (on-bound dims)');
for i = 1:numel(res.summary)
    s = res.summary(i);
    R = res.ctrl.(s.ctrl);
    g = cell(1, numel(s.comp_x));
    for j = 1:numel(s.comp_x)
        g{j} = sprintf('%s=%.4g', R.S.names{j}, s.comp_x(j));
    end
    if isempty(s.on_bound_dims)
        ob = 'none';
    else
        ob = strjoin(s.on_bound_dims, ',');
    end
    fprintf('%-12s %4d %6d %8.1f %8.4f +/- %6.4f %4d/%d  %s (%s)\n', s.ctrl, s.n_vars, s.n_evals, ...
        s.runtime_s, s.hv_mean, s.hv_std, s.front_size - s.front_penalised, s.front_size, ...
        strjoin(g, ' '), ob);
    f = cell(1, numel(s.comp_f));
    for j = 1:numel(s.comp_f)
        f{j} = sprintf('%s=%.4g', R.obj_names{j}, s.comp_f(j));
    end
    fprintf('%-12s objectives: %s\n', '', strjoin(f, ' '));
end
fprintf('(front = non-penalised/total members of the merged front; total %.1f s)\n', res.runtime_s);
end

function v = getf(s, name, default)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
