function res = run_robustness(gains_table, cfg)
%RUN_ROBUSTNESS  Scenario battery with corrected (post-event) metrics.
%   res = run_robustness(gains_table)        study_config() defaults
%   res = run_robustness(gains_table, cfg)   cfg from study_config (.rob used)
%
%   gains_table: struct with one field per controller, each either a gain
%   row in sim_boost order (outer loop = P.KPV/P.KIV) or a struct with
%   fields gains, KPV, KIV.  The result of run_mopso_study is accepted as
%   well (its gains_table is used).  [] = notebook baseline gains.
%
%   Settings (cfg.rob, see study_config): controllers ({} = all fields of
%   the table), plant ('switched' default, nsub 50, pwm 'center'; if this
%   sim_boost rejects opts.pwm the battery falls back to trailing-edge PWM
%   with a warning), ff_mode ('measured'; 'oracle' to compare), delay
%   (default = tuning delay cfg.sim.delay), extra_delay, mc_seeds (30),
%   noise, parasitics, scenarios ([] = drv_rob_scenarios), scenario_names
%   ({} = all), keep_traces ({'nominal'}), T_end ([] = P.T_end).
%
%   Metrics (metrics_event, band cfg.band_pct, event time of the scenario;
%   switched plant: envelope Vo_min/Vo_max for overshoot, peak and ripple):
%     startup_settling_ms  startup_overshoot_pct  pre_offset_V
%     pre_ripple_pp_V (switched plant only, NaN on the averaged plant)
%     event_peak_dev_V  recovery_ms  post_offset_V
%   NaN = never settled / never recovered / not applicable.
%
%   res fields: controllers, scenarios (name, label, t_evt, mc, n_runs,
%   needs_switched), metrics, metric_units, mean, std (nS x nC x nM; std is
%   NaN for deterministic scenarios), n_nan (NaN runs per entry), raw{s,c}
%   (runs x nM), status{s,c} ('ok' | 'skipped: ...' | 'failed: ...'),
%   diverged (nS x nC), gains, opts (base sim options), warnings, traces
%   .<scenario>.<ctrl> (t Vo iL d Vo_min Vo_max iL_min iL_max), n_sims,
%   runtime_s.

if nargin < 2
    cfg = [];
end
cfg = study_config(cfg);
P = cfg.P;
rob = cfg.rob;
t0 = tic;
res.warnings = {};

% gains
if nargin < 1 || isempty(gains_table)
    gains_table = struct();
    for i = 1:numel(cfg.controllers)
        gains_table.(cfg.controllers{i}) = P.gains_baseline.(cfg.controllers{i});
    end
elseif isfield(gains_table, 'gains_table')
    gains_table = gains_table.gains_table;
end
ctrls = rob.controllers;
if isempty(ctrls)
    ctrls = fieldnames(gains_table).';
end
if ischar(ctrls)
    ctrls = {ctrls};
end
gains = struct();
for i = 1:numel(ctrls)
    ctrls{i} = drv_ctrl_name(ctrls{i});
    if ~isfield(gains_table, ctrls{i})
        error('run_robustness:gains', 'no gains for %s in the table', ctrls{i});
    end
    gains.(ctrls{i}) = gain_entry(gains_table.(ctrls{i}), P);
end

% base options
switched = strcmp(rob.plant, 'switched');
base = struct('ff_mode', rob.ff_mode, 'delay', rob.delay, 'plant', rob.plant);
if ~isempty(rob.T_end)
    base.T_end = rob.T_end;
end
if switched
    base.nsub = rob.nsub;
    base.pwm = rob.pwm;
    try
        b1 = base;
        b1.T_end = 3 * P.Ts;
        sim_boost('cascade_pi', [], P, b1);
    catch e
        if ~isempty(strfind(lower(e.message), 'pwm')) || ~isempty(strfind(e.identifier, 'pwm'))
            base = rmfield(base, 'pwm');
            res.warnings{end + 1} = sprintf(['sim_boost rejected opts.pwm = ''%s'' (%s); ' ...
                'falling back to trailing-edge PWM'], rob.pwm, e.message);
            warning('run_robustness:pwm', '%s', res.warnings{end});
        else
            rethrow(e);
        end
    end
end

% scenarios
scen = rob.scenarios;
if isempty(scen)
    scen = drv_rob_scenarios(P, rob);
end
if ~isempty(rob.scenario_names)
    keep = ismember({scen.name}, rob.scenario_names);
    scen = scen(keep);
end
mnames = {'startup_settling_ms', 'startup_overshoot_pct', 'pre_offset_V', 'pre_ripple_pp_V', ...
    'event_peak_dev_V', 'recovery_ms', 'post_offset_V'};
units = {'ms', '%', 'V', 'V', 'V', 'ms', 'V'};
nS = numel(scen);
nC = numel(ctrls);
nM = numel(mnames);
res.controllers = ctrls;
res.metrics = mnames;
res.metric_units = units;
res.mean = NaN(nS, nC, nM);
res.std = NaN(nS, nC, nM);
res.n_nan = zeros(nS, nC, nM);
res.raw = cell(nS, nC);
res.status = repmat({'ok'}, nS, nC);
res.diverged = false(nS, nC);
res.traces = struct();
n_sims = 0;
sc = struct('name', {}, 'label', {}, 't_evt', {}, 'mc', {}, 'n_runs', {}, 'needs_switched', {});

for s = 1:nS
    S = scen(s);
    if S.mc
        seeds = 1:rob.mc_seeds;
    else
        seeds = 0;
    end
    sc(s) = struct('name', S.name, 'label', S.label, 't_evt', S.t_evt, 'mc', S.mc, ...
        'n_runs', numel(seeds), 'needs_switched', S.needs_switched);
    if rob.verbose
        fprintf('[%s] %s\n', S.name, S.label);
    end
    for c = 1:nC
        name = ctrls{c};
        if S.needs_switched && ~switched
            res.status{s, c} = 'skipped: needs the switched plant';
            if rob.verbose
                fprintf('  %-12s skipped (needs rob.plant = ''switched'')\n', name);
            end
            continue;
        end
        opts = drv_merge(base, S.opts);
        opts.load = S.load;
        opts.vin = S.vin;
        opts.KPV = gains.(name).KPV;
        opts.KIV = gains.(name).KIV;
        if ~isempty(fieldnames(S.pplant))
            opts.Pplant = drv_merge(P, S.pplant);
        end
        ev = struct('t_evt', S.t_evt, 'band_pct', cfg.band_pct);
        raw = NaN(numel(seeds), nM);
        div = false;
        for k = 1:numel(seeds)
            opts.seed = seeds(k);
            out = drv_run_batch(name, gains.(name).gains, P, opts, cfg);
            n_sims = n_sims + 1;
            if ~out.ok
                res.status{s, c} = ['failed: ' out.err];
                continue;
            end
            if any(~isfinite(out.Vo)) || max(abs(out.Vo)) > cfg.penalty.diverge_factor * P.Vref
                div = true;
            end
            evk = ev;
            has_env = ~isempty(out.Vo_min) && ~isempty(out.Vo_max);
            if has_env
                evk.Vo_min = out.Vo_min;
                evk.Vo_max = out.Vo_max;
            end
            m = metrics_event(out.t, out.Vo, P, evk);
            row = zeros(1, nM);
            for j = 1:nM
                row(j) = m.(mnames{j});
            end
            if ~has_env
                row(strcmp(mnames, 'pre_ripple_pp_V')) = NaN;
            end
            raw(k, :) = row;
            if k == 1 && any(strcmp(S.name, rob.keep_traces))
                res.traces.(S.name).(name) = struct('t', out.t, 'Vo', out.Vo, 'iL', out.iL, ...
                    'd', out.d, 'Vo_min', out.Vo_min, 'Vo_max', out.Vo_max, ...
                    'iL_min', out.iL_min, 'iL_max', out.iL_max);
            end
        end
        res.raw{s, c} = raw;
        res.diverged(s, c) = div;
        fin = ~isnan(raw);
        res.n_nan(s, c, :) = reshape(sum(~fin, 1), 1, 1, nM);
        for j = 1:nM
            v = raw(fin(:, j), j);
            if ~isempty(v)
                res.mean(s, c, j) = mean(v);
                if S.mc && numel(v) > 1
                    res.std(s, c, j) = std(v);
                elseif S.mc
                    res.std(s, c, j) = 0;
                end
            end
        end
        if rob.verbose
            fprintf('  %-12s ts %7.2f ms  OS %6.2f %%  off %6.3f V  rip %6.3f V  peak %6.3f V  rec %7.2f ms  off+ %6.3f V%s\n', ...
                name, res.mean(s, c, 1), res.mean(s, c, 2), res.mean(s, c, 3), res.mean(s, c, 4), ...
                res.mean(s, c, 5), res.mean(s, c, 6), res.mean(s, c, 7), flag_txt(div, S.mc, raw));
        end
    end
end
res.scenarios = sc;
res.values = res.mean;
res.gains = gains;
res.opts = base;
res.band_pct = cfg.band_pct;
res.n_sims = n_sims;
res.runtime_s = toc(t0);
if rob.verbose
    fprintf('robustness battery: %d simulations, %.1f s\n', n_sims, res.runtime_s);
end
end

% ---------------------------------------------------------------------------
function g = gain_entry(v, P)
if isstruct(v)
    g.gains = double(v.gains(:).');
    g.KPV = getf(v, 'KPV', P.KPV);
    g.KIV = getf(v, 'KIV', P.KIV);
else
    g.gains = double(v(:).');
    g.KPV = P.KPV;
    g.KIV = P.KIV;
end
end

function s = flag_txt(div, mc, raw)
s = '';
if mc
    s = sprintf('  (MC n=%d)', size(raw, 1));
end
if div
    s = [s '  DIVERGED'];
end
end

function v = getf(s, name, default)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
