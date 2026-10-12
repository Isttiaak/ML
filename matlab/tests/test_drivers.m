function results = test_drivers()
%TEST_DRIVERS  Tiny-budget end-to-end tests of the study drivers.
%   results = test_drivers() prints one PASS/FAIL line per check, returns a
%   struct array (name, passed, detail) and errors if any check failed.
%   Covers study_config, drv_sim_batch (== sim_boost), objective_boost
%   (legacy == Python notebook metrics, corrected penalties, worst-case
%   aggregation, Simulink-emulation backend), run_mopso_study (2 controllers,
%   6 particles x 4 iterations, 2 seeds, both objective sets, common HV
%   normalisation), run_robustness (2 controllers, mc_seeds 3), LaTeX
%   escaping and make_paper_figures (dry run + real render when a graphics
%   toolkit exists).  Runtime: about 1.5 min in Octave 8.4.
here = fileparts(mfilename('fullpath'));
addpath(fileparts(here));
addpath(here);
t0 = tic;
R = struct('name', {}, 'passed', {}, 'detail', {});
ctx = struct();
ctx.tmp = tempname();
mkdir(ctx.tmp);
ctx.refdir = fullfile(here, 'ref');

[R, ctx] = run_check(R, ctx, 'config_defaults_and_overrides', @t_config);
[R, ctx] = run_check(R, ctx, 'batch_sim_equals_sim_boost', @t_batch);
[R, ctx] = run_check(R, ctx, 'legacy_objectives_match_python', @t_legacy_obj);
[R, ctx] = run_check(R, ctx, 'corrected_objectives_and_penalties', @t_corrected_obj);
[R, ctx] = run_check(R, ctx, 'worst_case_max_aggregation', @t_worst_case);
[R, ctx] = run_check(R, ctx, 'simulink_emulate_backend_parity', @t_simulink_emulate);
[R, ctx] = run_check(R, ctx, 'mopso_study_corrected_tiny', @t_study_corrected);
[R, ctx] = run_check(R, ctx, 'mopso_study_legacy_tiny', @t_study_legacy);
[R, ctx] = run_check(R, ctx, 'robustness_battery_switched', @t_rob_switched);
[R, ctx] = run_check(R, ctx, 'robustness_battery_avg_and_oracle', @t_rob_avg);
[R, ctx] = run_check(R, ctx, 'latex_escaping_and_format', @t_latex);
[R, ctx] = run_check(R, ctx, 'paper_figures', @t_figures);

try
    rmdir(ctx.tmp, 's');
catch
end
n_pass = sum([R.passed]);
fprintf('test_drivers: %d/%d checks passed (%.1f s)\n', n_pass, numel(R), toc(t0));
results = R;
if n_pass < numel(R)
    error('test_drivers:failed', '%d check(s) failed', numel(R) - n_pass);
end
end

% ---------------------------------------------------------------------------
function [R, ctx] = run_check(R, ctx, name, fn)
t1 = tic;
try
    [ok, detail, ctx] = fn(ctx);
catch e
    ok = false;
    detail = sprintf('ERROR %s', e.message);
    if ~isempty(e.stack)
        detail = sprintf('%s (%s:%d)', detail, e.stack(1).name, e.stack(1).line);
    end
end
if ok
    tag = 'PASS';
else
    tag = 'FAIL';
end
fprintf('%s %s: %s [%.1f s]\n', tag, name, detail, toc(t1));
R(end + 1) = struct('name', name, 'passed', logical(ok), 'detail', detail);
end

function cfg = tiny_cfg(varargin)
cfg = study_config(varargin{:});
cfg = drv_setpath(cfg, 'controllers', {'cascade_pi', 'smc'});
cfg = drv_setpath(cfg, 'mopso.n_particles', 6);
cfg = drv_setpath(cfg, 'mopso.n_iter', 4);
cfg = drv_setpath(cfg, 'mopso_ctrl', struct());
cfg = drv_setpath(cfg, 'hv.samples', 2e4);
cfg = drv_setpath(cfg, 'hv.hist_samples', 5e3);
cfg = drv_setpath(cfg, 'verbose', false);
end

% ---------------------------------------------------------------------------
function [ok, detail, ctx] = t_config(ctx)
c = study_config();
okc = isequal(c.controllers, {'fcsmpc', 'cascade_pi', 'cascade_pid', 'smc', 'single_pi'}) ...
    && strcmp(c.objective_set, 'corrected') && c.tune_outer && strcmp(c.sim.ff_mode, 'measured') ...
    && c.sim.delay == 1 && strcmp(c.sim.plant, 'avg') && numel(c.scenarios) == 1 ...
    && abs(c.scenarios(1).t_evt - 0.05) < 1e-15 && isequal(c.seeds, 1:5) ...
    && c.mopso.n_particles == 20 && c.mopso.n_iter == 30 && c.mopso.archive_size == 60 ...
    && strcmp(c.backend, 'matlab') && c.rob.delay == 1 && strcmp(c.rob.pwm, 'center');
dims = zeros(1, 5);
okd = true;
for i = 1:5
    S = drv_search_space(c.controllers{i}, c);
    dims(i) = numel(S.lb);
    okd = okd && all(S.lb < S.ub) && all(S.lb(S.log_scale) > 0) && all(S.x0 >= S.lb) ...
        && all(S.x0 <= S.ub) && ~S.x0_clipped && numel(S.names) == dims(i);
end
okd = okd && isequal(dims, [3 4 6 4 2]);
c2 = study_config('controllers', 'pid', 'mopso.n_iter', 4, 'tune_outer', false, ...
    'scenario_set', 'worst_case');
S2 = drv_search_space('cascade_pid', c2);
oko = isequal(c2.controllers, {'cascade_pid'}) && c2.mopso.n_iter == 4 && c2.mopso.n_particles == 20 ...
    && numel(S2.lb) == 4 && numel(c2.scenarios) == 5;
[G, KPV, KIV] = drv_decode([1 2 3 4; 5 6 7 8], 'cascade_pid', c2);
oko = oko && isequal(G, [1 2 3 4; 5 6 7 8]) && all(KPV == c2.P.KPV) && all(KIV == c2.P.KIV);
[G, KPV, KIV] = drv_decode([0.1 2 0.3 40], 'smc', c);
oko = oko && isequal(G, [0.1 2]) && KPV == 0.3 && KIV == 40;
L = study_config('legacy');
SL = drv_search_space('cascade_pi', L);
okl = strcmp(L.objective_set, 'legacy') && ~L.tune_outer && strcmp(L.sim.ff_mode, 'oracle') ...
    && L.sim.delay == 0 && L.mopso.legacy && isequal(L.seeds, 42) && isequal(SL.lb, [0.005 1]) ...
    && isequal(SL.ub, [0.15 80]) && ~any(SL.log_scale) && L.mopso_ctrl.fcsmpc.n_particles == 15;
c3 = study_config(c2);                       % round trip keeps overrides
okr = isequal(c3.controllers, {'cascade_pid'}) && numel(c3.scenarios) == 5;
okerr = false;
try
    study_config('controllers', {'nope'});
catch
    okerr = true;
end
ok = okc && okd && oko && okl && okr && okerr;
detail = sprintf('defaults %d, dims %s, overrides %d, legacy preset %d, round trip %d, bad ctrl error %d', ...
    okc, mat2str(dims), oko, okl, okr, okerr);
end

function [ok, detail, ctx] = t_batch(ctx)
P = boost_params();
ctrls = {'fcsmpc', 'cascade_pi', 'cascade_pid', 'smc', 'single_pi'};
Gs = {[0.5; 1.48], [0.03 20; 0.053 1], [0.033 22 8e-6 1e-4; 0.05 1 1e-7 3.7e-4], ...
    [0.4 0.5; 0.32 2], [0.05 5; 1e-3 0.1]};
o = struct('ff_mode', 'measured', 'delay', 2, 'noise_std_v', 0.05, 'noise_std_i', 0.02, ...
    'noise_std_io', 0.02, 'noise_std_vin', 0.05, 'adc_bits', 10, 'seed', 3, 'KPV', [0.15; 0.4], ...
    'KIV', [12; 80], 'vin', struct('kind', 'step', 'pct', -0.2, 't_step', 0.06));
o2 = struct('ff_mode', 'oracle', 'delay', 0);
maxd = 0;
t_b = 0;
t_l = 0;
nl = 0;
for c = 1:numel(ctrls)
    for k = 1:2
        if k == 1
            oo = o;
        else
            oo = o2;
        end
        tic;
        b = drv_sim_batch(ctrls{c}, Gs{c}, P, oo);
        t_b = t_b + toc;
        for i = 1:2
            oi = oo;
            if isfield(oo, 'KPV')
                oi.KPV = oo.KPV(i);
                oi.KIV = oo.KIV(i);
            end
            tic;
            r = sim_boost(ctrls{c}, Gs{c}(i, :), P, oi);
            t_l = t_l + toc;
            nl = nl + 1;
            dd = [abs(r.Vo - b.Vo(:, i)); abs(r.iL - b.iL(:, i)); abs(r.d - b.d(:, i))];
            dd(isnan(r.Vo) & isnan(b.Vo(:, i))) = 0;
            maxd = max(maxd, max(dd));
        end
    end
end
ctx.t_sim_boost = t_l / nl;
ok = maxd <= 1e-9;
detail = sprintf(['max |batch - sim_boost| = %.3g over 5 controllers x 2 option sets x 2 rows ' ...
    '(exact: %d); sim_boost %.3f s/run, batch of 2 %.3f s'], maxd, maxd == 0, t_l / nl, t_b / 10);
end

function [ok, detail, ctx] = t_legacy_obj(ctx)
J = core_read_json_exact(fullfile(ctx.refdir, 'python_reference.json'));
cfg = study_config('legacy');
ctrls = cfg.controllers;
key = {'Settling_ms_', 'Overshoot___', 'SSError_V_', 'LoadReg__V_'};
md = 0;
for c = 1:numel(ctrls)
    for which = 1:2
        if which == 1
            g = J.gains_baseline.(ctrls{c})(:).';
            m = J.metrics.base.(ctrls{c});
        else
            g = J.gains_mopso_python.(ctrls{c})(:).';
            m = J.metrics.opt.(ctrls{c});
        end
        ref = zeros(1, 4);
        for k = 1:4
            ref(k) = m.(key{k});
        end
        F = objective_boost(g, ctrls{c}, [], cfg);
        md = max(md, max(abs(F - ref)));
    end
end
ok = md <= 1e-4;          % values are rounded to 1e-4 (SMC tanh may differ by 1 ulp)
detail = sprintf('max |F - notebook metric| = %.3g over 5 controllers x (baseline, notebook MOPSO gains)', md);
end

function [ok, detail, ctx] = t_corrected_obj(ctx)
cfg = study_config();
P = cfg.P;
pen = cfg.penalty;
rows = struct();
okall = true;
for c = 1:numel(cfg.controllers)
    name = cfg.controllers{c};
    S = drv_search_space(name, cfg);
    X = [S.x0; S.x0 .* (0.7 + 0.6 * ((1:numel(S.x0)) / numel(S.x0)))];
    X = min(max(X, S.lb), S.ub);
    [F, aux] = objective_boost(X, name, P, cfg);
    rows.(name) = F;
    nobj = 4 + strcmp(name, 'fcsmpc');
    okall = okall && isequal(size(F), [2 nobj]) && all(isfinite(F(:))) && isequal(size(aux.flags), [2 1]) ...
        && numel(aux.obj_names) == nobj;
end
% notebook baselines: FCS-MPC never settles (ripple > band), single PI diverges
okm = rows.fcsmpc(1, 1) == pen.settling_ms && rows.fcsmpc(1, 5) > 0 && rows.fcsmpc(1, 5) < P.fs / 2e3;
bad = [pen.settling_ms pen.overshoot_pct pen.peak_dev_V pen.recovery_ms];
oks = isequal(rows.single_pi(1, :), bad);
% a failing simulation (NaN gain) gets the diverged row and flag 2
[Fn, auxn] = objective_boost([NaN 20 0.15 12], 'cascade_pi', P, cfg);
okn = isequal(Fn, bad) && auxn.flags == 2;
% every non-diverged row dominates the diverged row
others = [rows.cascade_pi; rows.cascade_pid; rows.smc; rows.fcsmpc(:, 1:4)];
okd = all(all(others <= bad, 2) & any(others < bad, 2));
% the notebook baseline cascade PI under the corrected set-up
okb = all(rows.cascade_pi(1, :) < [pen.settling_ms 100 10 pen.recovery_ms]);
% optional extra objectives (metrics_event fields) are appended after the base set
ce = study_config('extra_objectives', {'event_iae_Vms', 'post_offset_V'});
Fe = objective_boost([0.03 20 0.15 12], 'cascade_pi', P, ce);
Fm = objective_boost([0.5 0.15 12], 'fcsmpc', P, ce);
Fd = objective_boost([NaN 20 0.15 12], 'cascade_pi', P, ce);
oke = isequal(size(Fe), [1 6]) && isequal(Fe(1:4), rows.cascade_pi(1, :)) && all(isfinite(Fe)) ...
    && isequal(size(Fm), [1 7]) && isequal(Fd, [bad 1e6 1e6]) ...
    && isequal(drv_objective_names('fcsmpc', ce), {'startup_settling_ms', 'startup_overshoot_pct', ...
    'event_peak_dev_V', 'recovery_ms', 'fsw_kHz', 'event_iae_Vms', 'post_offset_V'});
ok = okall && okm && oks && okn && okd && okb && oke;
detail = sprintf(['shapes/finite %d, MPC settling penalty %d (fsw %.2f kHz), single PI diverged row %d, ' ...
    'NaN gain -> penalty %d, penalty row dominated %d, extra objectives %d; baseline PI F = %s'], okall, okm, ...
    rows.fcsmpc(1, 5), oks, okn, okd, oke, mat2str(rows.cascade_pi(1, :), 4));
end

function [ok, detail, ctx] = t_worst_case(ctx)
c1 = study_config();
c5 = study_config('scenario_set', 'worst_case');
S = drv_search_space('smc', c1);
X = [S.x0; 0.3 3 0.2 5; 0.6 0.2 0.05 50];
F1 = objective_boost(X, 'smc', [], c1);
[F5, aux] = objective_boost(X, 'smc', [], c5);
okmax = isequal(F5, max(aux.Fs, [], 3)) && isequal(size(aux.Fs), [3 4 5]);
oknom = isequal(aux.Fs(:, :, 1), F1) && all(F5(:) >= F1(:));
t_evts = [c5.scenarios.t_evt];
ok = okmax && oknom && isequal(t_evts, [0.05 0.05 0.05 0.06 0.05]) ...
    && c5.scenarios(5).sim.Pplant.L == 0.8 * c5.P.L;
detail = sprintf('F = max over 5 scenarios %d, scenario 1 == nominal-only objective %d, event times %s', ...
    okmax, oknom, mat2str(t_evts));
end

function [ok, detail, ctx] = t_simulink_emulate(ctx)
if exist('sim_boost_simulink', 'file') ~= 2
    ok = true;
    detail = 'sim_boost_simulink.m not on the path: check skipped';
    return;
end
c = study_config('backend', 'simulink', 'simcfg', struct('backend', 'emulate'));
cm = study_config();
S = drv_search_space('cascade_pi', cm);
X = [S.x0; 0.07 0.75 0.09 2.8];
tic;
Fs = objective_boost(X, 'cascade_pi', [], c);
ts = toc;
Fm = objective_boost(X, 'cascade_pi', [], cm);
okp = max(abs(Fs(:) - Fm(:))) <= 1e-9;
% per-row outer gains really reach the runs
Xs = [S.x0; S.x0];
Xs(2, 3) = 0.5;
F2 = objective_boost(Xs, 'cascade_pi', [], c);
okk = any(F2(1, :) ~= F2(2, :));
ok = okp && okk;
detail = sprintf('simulink(emulate) vs matlab objectives max diff %.3g, per-row KPV used %d (%.2f s)', ...
    max(abs(Fs(:) - Fm(:))), okk, ts);
end

function [ok, detail, ctx] = t_study_corrected(ctx)
cfg = tiny_cfg('seeds', [1 2], 'out_file', fullfile(ctx.tmp, 'study.mat'));
tic;
res = run_mopso_study(cfg);
rt = toc;
okall = true;
msg = {};
for c = 1:2
    name = cfg.controllers{c};
    Rc = res.ctrl.(name);
    S = Rc.S;
    runs = Rc.runs;
    m = numel(Rc.obj_names);
    oks = numel(runs) == 2 && Rc.n_evals == 48 && all([runs.n_evals] == 24);
    okf = true;
    okinit = true;
    allF = [];
    allp = [];
    for k = 1:2
        okf = okf && all(isfinite(runs(k).arch_F(:))) && all(isfinite(runs(k).F_all(:))) ...
            && size(runs(k).arch_F, 2) == m;
        okinit = okinit && max(abs(runs(k).X_all(1, :) - S.x0) ./ S.x0) < 1e-12;
        allF = [allF; runs(k).arch_F]; %#ok<AGROW>
        allp = [allp; runs(k).penalised]; %#ok<AGROW>
    end
    base = allF(~allp, :);
    if isempty(base)
        base = allF;
    end
    oknorm = max(abs(Rc.norm.ideal - min(base, [], 1))) < 1e-12 ...
        && max(abs(Rc.norm.nadir - max(base, [], 1))) < 1e-12 && all(Rc.norm.ref == 1.1);
    Fn = (runs(1).arch_F - Rc.norm.ideal) ./ Rc.norm.span;
    hv1 = hypervolume_mc(Fn, Rc.norm.ref, cfg.hv.samples, cfg.hv.seed);
    okhv = abs(hv1 - Rc.hv(1)) < 1e-12 && all(Rc.hv >= 0 & Rc.hv <= 1.1^m) ...
        && isequal(size(Rc.hv_hist), [4 2]) && all(isfinite(Rc.hv_hist(:))) ...
        && all(Rc.hv_hist(end, :) >= Rc.hv_hist(1, :) - 0.05) && abs(Rc.hv_mean - mean(Rc.hv)) < 1e-15;
    Fv = (base - Rc.norm.ideal) ./ Rc.norm.span;
    okhv = okhv && all(Fv(:) >= -1e-12 & Fv(:) <= 1 + 1e-12);
    mg = Rc.merged;
    okm = all(mg.x >= S.lb & mg.x <= S.ub) && numel(mg.margin) == numel(S.lb) ...
        && all(mg.margin >= 0 & mg.margin <= 0.5) && ~mg.penalised(mg.comp_idx) ...
        && isequal(res.gains_table.(name).gains, mg.x(1:S.ngains)) ...
        && res.gains_table.(name).KPV == mg.x(S.ngains + 1) && numel(runs(1).comp_margin) == numel(S.lb) ...
        && all(size(runs(1).bound_frac) == [1 numel(S.lb)]);
    okall = okall && oks && okf && okinit && oknorm && okhv && okm;
    msg{end + 1} = sprintf('%s: evals %d HV %.3f+/-%.3f comp margin min %.2f [%d%d%d%d%d%d]', name, ...
        Rc.n_evals, Rc.hv_mean, Rc.hv_std, min(mg.margin), oks, okf, okinit, oknorm, okhv, okm); %#ok<AGROW>
end
L = load(cfg.out_file);
oksave = isfield(L, 'res') && isfield(L.res.ctrl, 'smc') && numel(res.summary) == 2;
ctx.study = res;
ok = okall && oksave;
detail = sprintf('%s; saved %d; %.1f s', strjoin(msg, '; '), oksave, rt);
end

function [ok, detail, ctx] = t_study_legacy(ctx)
cfg = tiny_cfg('legacy', 'seeds', [42 43]);
tic;
res = run_mopso_study(cfg);
rt = toc;
ok = true;
for c = 1:2
    Rc = res.ctrl.(cfg.controllers{c});
    ok = ok && isequal(Rc.obj_names, {'settling_ms', 'overshoot_pct', 'ss_error_V', 'load_reg_V'}) ...
        && Rc.n_evals == 48 && all(reshape(isfinite(vertcat(Rc.runs.arch_F)), [], 1)) ...
        && numel(Rc.S.lb) == 2 && Rc.mopso_opts.legacy && all(isfinite(Rc.hv));
end
detail = sprintf('legacy objectives, notebook bounds, 2 vars, legacy MOPSO: %d (%.1f s)', ok, rt);
end

function [ok, detail, ctx] = t_rob_switched(ctx)
cfg = study_config('rob.mc_seeds', 3, 'rob.nsub', 20, 'rob.verbose', false);
gt = struct('cascade_pi', struct('gains', [0.07 0.75], 'KPV', 0.09, 'KIV', 2.8), ...
    'smc', [0.4 0.5]);
tic;
r = run_robustness(gt, cfg);
rt = toc;
nS = numel(r.scenarios);
names = {r.scenarios.name};
expect = {'nominal', 'load_m55', 'load_release', 'load_ramp', 'vin_p20', 'vin_m20', 'L_m20', ...
    'L_p20', 'C_m20', 'C_p20', 'parasitics', 'delay_plus', 'noise_mc'};
oks = isequal(names, expect) && isequal(size(r.mean), [13 2 7]) && isequal(r.controllers, {'cascade_pi', 'smc'});
imc = find([r.scenarios.mc]);
okmc = isequal(imc, 13) && r.scenarios(13).n_runs == 3 && all(isfinite(r.std(13, :, 5))) ...
    && size(r.raw{13, 1}, 1) == 3 && all(isnan(reshape(r.std(1:12, :, :), [], 1)));
irip = strcmp(r.metrics, 'pre_ripple_pp_V');
okrip = all(isfinite(r.mean(1, :, irip))) && all(r.mean(1, :, irip) > 0.01);
okpar = all(strcmp(r.status(11, :), 'ok')) && all(isfinite(r.mean(11, :, 5)));
okg = r.gains.smc.KPV == cfg.P.KPV && r.gains.cascade_pi.KIV == 2.8;
okt = isfield(r.traces, 'nominal') && isfield(r.traces.nominal, 'smc') ...
    && numel(r.traces.nominal.smc.Vo_min) == cfg.P.N;
P = cfg.P;
Rmin = zeros(1, nS);
okfeas = true;
sp = drv_rob_scenarios(P, cfg.rob);
t = (0:P.N - 1)' * P.Ts;
for s = 1:nS
    Rmin(s) = min(load_profile(t, sp(s).load));
    vmin = min(vin_profile(t, sp(s).vin));
    okfeas = okfeas && Rmin(s) >= P.Vref^2 / (vmin * P.IL_MAX) - 1e-9;
end
okev = isequal([r.scenarios([5 6]).t_evt], [0.06 0.06]);
ok = oks && okmc && okrip && okpar && okg && okt && okfeas && okev && strcmp(r.opts.pwm, 'center') ...
    && r.n_sims == 2 * (12 + 3);
ctx.rob = r;
detail = sprintf(['13 scenarios %d, MC mean/std %d, envelope ripple %.3f/%.3f V %d, parasitics %d, ' ...
    'gains %d, traces %d, feasible (R >= Vref^2/(Vin*IL_MAX)) %d, Vin events 60 ms %d, %d sims in %.1f s ' ...
    '(%.2f s/sim, nsub 20)'], oks, okmc, r.mean(1, 1, irip), r.mean(1, 2, irip), okrip, okpar, okg, okt, ...
    okfeas, okev, r.n_sims, rt, rt / r.n_sims);
end

function [ok, detail, ctx] = t_rob_avg(ctx)
cfg = study_config('rob.plant', 'avg', 'rob.mc_seeds', 2, 'rob.verbose', false, ...
    'rob.scenario_names', {'nominal', 'parasitics', 'noise_mc'});
gt = struct('cascade_pi', [0.03 20]);
r = run_robustness(gt, cfg);
irip = strcmp(r.metrics, 'pre_ripple_pp_V');
oka = numel(r.scenarios) == 3 && strncmp(r.status{2, 1}, 'skipped', 7) && all(isnan(r.mean(2, 1, :))) ...
    && isnan(r.mean(1, 1, irip)) && isfinite(r.mean(1, 1, 1)) && r.scenarios(3).n_runs == 2;
% oracle feed-forward option (true R and Vin) for comparison with 'measured'
cfg2 = drv_setpath(cfg, 'rob.ff_mode', 'oracle');
cfg2.rob.scenario_names = {'nominal'};
r2 = run_robustness(gt, cfg2);
d = abs(r2.mean(1, 1, 5) - r.mean(1, 1, 5));
oko = strcmp(r2.opts.ff_mode, 'oracle') && strcmp(r.opts.ff_mode, 'measured') ...
    && all(isfinite(r2.mean(1, 1, [1 2 5])));
% default (baseline) gains table
cfg3 = drv_setpath(cfg, 'rob.scenario_names', {'nominal'});
cfg3.controllers = {'cascade_pid', 'smc'};
r3 = run_robustness([], cfg3);
okb = isequal(r3.controllers, {'cascade_pid', 'smc'}) && isequal(r3.gains.smc.gains, cfg.P.gains_baseline.smc);
ok = oka && oko && okb;
detail = sprintf('avg plant: parasitics skipped, ripple NaN %d; oracle vs measured peak dev differ by %.3g V %d; baseline table %d', ...
    oka, d, oko, okb);
end

function [ok, detail, ctx] = t_latex(ctx)
okesc = strcmp(drv_latex_escape('50% a_b & c #1 $x$ {y} ~^\'), ...
    '50\% a\_b \& c \#1 \$x\$ \{y\} \textasciitilde{}\textasciicircum{}\textbackslash{}');
r = ctx.rob;
r.scenarios(1).label = 'Load -50% (base) a_b & c #1';
r.mean(2, 1, 1) = NaN;                                   % force a '--'
r.n_nan(13, 1, 6) = 1;                                    % force a '*'
r.diverged(3, 2) = true;                                  % force a dagger
f = fullfile(ctx.tmp, 'rob.tex');
str = robustness_to_latex(r, f);
back = fileread(f);
unesc = regexp(str, '(?<!\\)%', 'once');
nrows = numel(regexp(str, '\\\\\n', 'match'));
okfmt = ~isempty(strfind(str, '\begin{table*}')) && ~isempty(strfind(str, '\footnotesize')) ...
    && ~isempty(strfind(str, 'Load -50\% (base) a\_b \& c \#1')) && ~isempty(strfind(str, '$\pm$')) ...
    && ~isempty(strfind(str, '--')) && ~isempty(strfind(str, '$^{*}$')) ...
    && ~isempty(strfind(str, 'SMC$^{\dagger}$')) && numel(strfind(str, '\dagger')) == 2 ...
    && ~isempty(strfind(str, '\end{table*}')) && strcmp(back, str);
% rows: header + 13 scenarios x 2 controllers
okrows = nrows == 1 + 13 * 2;
% precision: ms/% with 2 decimals, V with 3
row1 = regexp(str, 'Cascade PI & ([^\n]*)\\\\', 'tokens', 'once');
cells = strtrim(strsplit(row1{1}, '&'));
okprec = ~isempty(regexp(cells{1}, '^\d+\.\d{2}$', 'once')) && ~isempty(regexp(cells{3}, '^\d+\.\d{3}$', 'once'));
% the notebook's unescaped table would fail the same check
nb = 'Load -50% (base) & Cascade-PI & 7.2500 \\';
oknb = ~isempty(regexp(nb, '(?<!\\)%', 'once'));
ok = okesc && isempty(unesc) && okfmt && okrows && okprec && oknb;
detail = sprintf('escape %d, no unescaped %% %d, format %d, rows %d, precision %d, notebook-style row detected %d (%d chars)', ...
    okesc, isempty(unesc), okfmt, okrows, okprec, oknb, numel(str));
end

function [ok, detail, ctx] = t_figures(ctx)
cfg = study_config('fig.dry_run', true, 'fig.dir', fullfile(ctx.tmp, 'figs'), 'controllers', {'cascade_pi', 'smc'});
figs = make_paper_figures(struct('mopso', ctx.study, 'rob', ctx.rob), cfg);
names = {figs.name};
expect = {'fig_vo', 'fig_il', 'fig_loadstep', 'fig_ripple', 'fig_pareto', 'fig_hv'};
okdry = isequal(names, expect) && all(cellfun(@isempty, {figs.files})) && all(cellfun(@isempty, {figs.skipped})) ...
    && isequal(size(figs(1).data.Y, 2), 2) && numel(figs(4).data.ripple_pp_V) == 2 ...
    && all(figs(5).data.F(:) >= -1e-12 & figs(5).data.F(:) <= 1 + 1e-12) && numel(figs(6).data.mean) == 2 ...
    && ~exist(cfg.fig.dir, 'dir');
% excluded controller stays out
okexc = ~any(strcmp(figs(1).data.ctrls, 'single_pi'));
render = 'not attempted';
okr = true;
tk = {};
try
    tk = available_graphics_toolkits();
catch
    tk = {'matlab'};
end
if ~isempty(tk)
    cfg.fig.dry_run = false;
    cfg.fig.formats = {'png'};
    cfg.fig.dpi = 150;
    tic;
    figs2 = make_paper_figures(struct('mopso', ctx.study, 'rob', ctx.rob), cfg);
    tr = toc;
    nfiles = 0;
    for i = 1:numel(figs2)
        for k = 1:numel(figs2(i).files)
            dd = dir(figs2(i).files{k});
            nfiles = nfiles + (numel(dd) == 1 && dd.bytes > 1000);
        end
    end
    okr = nfiles == 6;
    render = sprintf('%d/6 png files rendered in %.1f s', nfiles, tr);
end
ok = okdry && okexc && okr;
detail = sprintf('dry run data %d, single_pi excluded %d, %s', okdry, okexc, render);
end
