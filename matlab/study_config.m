function cfg = study_config(varargin)
%STUDY_CONFIG  Configuration of the corrected MOPSO tuning / robustness study.
%   cfg = study_config()                    corrected methodology (default)
%   cfg = study_config('legacy')            notebook set-up (cells 1-3): notebook
%                                           bounds and budgets, legacy objectives,
%                                           oracle feed-forward, no delay, legacy
%                                           MOPSO preset, seed 42, random init
%   cfg = study_config(cfg0)                fill the missing fields of cfg0
%   cfg = study_config(..., 'path', value, ...)   overrides, nested with dots,
%         e.g. study_config('controllers', {'smc'}, 'mopso.n_iter', 4)
%   The first argument may be 'corrected', 'legacy' or a struct; the rest are
%   name/value pairs applied last.
%
%   Fields (defaults of the corrected study):
%   P               boost_params()
%   controllers     {'fcsmpc','cascade_pi','cascade_pid','smc','single_pi'}
%   vars.<ctrl>     decision variables of the controller (sim_boost gain
%                   order): names, lb, ub, log_scale.  WIDE bounds, searched
%                   in log10 where log_scale is true:
%                     fcsmpc      lsw   [1e-3 100] log
%                     cascade_pi  Kp_i  [1e-3 0.5] log   Ki_i [1e-2 1e3] log
%                     cascade_pid as cascade_pi + Kd_i [1e-9 1e-4] log,
%                                 Tf [2.5e-5 5e-3] log
%                     smc         K_s   [0.01 0.9] lin   phi  [0.01 20] log
%                     single_pi   Kp_v  [1e-4 0.5] log   Ki_v [1e-2 1e3] log
%                   (the notebook used [0.005 0.15] x [1 80] for the cascade PI
%                   etc. and its optima sat on those bounds)
%   tune_outer      true: append the outer voltage-PI gains outer_vars to every
%                   controller that has the outer loop (all but single_pi)
%   outer_vars      KPV [0.01 5] log, KIV [0.1 1e4] log (widened from the
%                   suggested [0.01 2] x [0.1 1e3]: FCS-MPC compromises hit
%                   KIV = 1e3 and KPV ~ 1.8 in exploratory runs)
%   objective_set   'corrected' (default) | 'legacy' (see objective_boost)
%   mpc_fsw_objective  true: FCS-MPC gets average switching frequency [kHz]
%                   as a 5th objective
%   extra_objectives   {} (default); corrected set only: metrics_event fields
%                   appended as further objectives, e.g. {'event_iae_Vms'}
%                   (penalises slow tails) or {'post_offset_V'}
%   sim             sim_boost options for tuning: ff_mode 'measured' (no
%                   oracle R/Vin), delay 1 (one-sample computation delay),
%                   plant 'avg'.  Any sim_boost option may be added.
%   scenario_set    'nominal' (default): the 20 -> 10 ohm step at 50 ms;
%                   'worst_case': nominal + 20 -> 8.9 ohm step + heavy-load
%                   start-up (10 ohm, released to 20 ohm at 50 ms) + Vin -20 %
%                   step at 60 ms (20 ohm) + L -20 % plant mismatch.  Tuning
%                   on 'nominal' only can over-fit (e.g. cascade PI with
%                   Ki_i ~ 0.05 that never settles when starting into 10 ohm).
%   scenarios       struct array (name, load, vin, t_evt, sim); built from
%                   scenario_set when empty.  sim holds per-scenario sim_boost
%                   options (e.g. Pplant).  Objectives are aggregated with
%                   scenario_agg = 'max' (worst case) over the scenarios.
%   band_pct        settling / recovery band, +/- % of Vref (2)
%   penalty         finite values used instead of NaN (see objective_boost):
%                   settling_ms 200, recovery_ms 200 (2*T_end), overshoot_pct
%                   200, peak_dev_V 3*Vref, fsw_kHz fs/1e3, extra 1e6 (extra
%                   objectives), diverge_factor 2
%   seeds           MOPSO seeds, one independent run each (1:5)
%   mopso           options passed to mopso(): n_particles 20, n_iter 30,
%                   archive_size 60, w 0.6, c1 = c2 = 1.5, crowding density,
%                   'reset_velocity' bounds, dedupe, ideal_point compromise.
%                   log_scale, seed, init_X and vectorized are set by the driver.
%   mopso_ctrl.<ctrl>  per-controller overrides of mopso (e.g. budget)
%   init_baseline   true: notebook baseline gains (+ P.KPV/P.KIV) are placed in
%                   the initial swarm
%   hv              ref 1.1 (in the common normalisation), samples 1e5,
%                   hist_samples 1e4, seed 1, history true
%   bound_tol_frac  0.01: a compromise gain closer than 1 % of the (log)
%                   search range to a bound is reported as 'on bound'
%   backend         'matlab' (sim_boost) | 'simulink' (sim_boost_simulink,
%                   one call per swarm with per-row opts, see drv_run_batch)
%   batch_sim       true: averaged-plant tuning uses drv_sim_batch (all
%                   particles at once, bit-identical to sim_boost)
%   simcfg          struct passed to sim_boost_simulink (model, use_parsim,
%                   fast_restart, ...; backend 'emulate' = Octave check)
%   out_file        '' or a .mat file for run_mopso_study results
%   verbose         true
%   rob             robustness battery settings (run_robustness): controllers
%                   ({} = all in the gains table), plant 'switched', pwm
%                   'center', nsub 50, ff_mode 'measured', delay (= sim.delay),
%                   extra_delay 1, mc_seeds 30, noise (20 mA iL, 50 mV Vo,
%                   20 mA io, 50 mV Vin, 10-bit ADC), parasitics (rL 50 mOhm,
%                   rC 20 mOhm, Ron 20 mOhm, VF 0.7 V), scenarios ([] = default
%                   battery, see drv_rob_scenarios), scenario_names ({} = all),
%                   keep_traces {'nominal'}, T_end (P.T_end)
%   fig             make_paper_figures settings: dir 'figures', formats {'png'},
%                   font 'Times New Roman', font_size 8, width_in 3.5, dpi 600,
%                   exclude {'single_pi'}, dry_run false

args = varargin;
preset = 'corrected';
user = struct();
if ~isempty(args)
    if ischar(args{1}) && any(strcmpi(args{1}, {'corrected', 'legacy'}))
        preset = lower(args{1});
        args = args(2:end);
    elseif isstruct(args{1}) || isempty(args{1})
        if isstruct(args{1})
            user = args{1};
            if isfield(user, 'preset') && ~isempty(user.preset)
                preset = lower(user.preset);
            end
        end
        args = args(2:end);
    end
end
if mod(numel(args), 2) ~= 0
    error('study_config:args', 'overrides must be name/value pairs');
end
for i = 1:2:numel(args)
    if strcmp(args{i}, 'preset')
        preset = lower(args{i + 1});
    end
end

cfg = defaults();
if strcmp(preset, 'legacy')
    cfg = drv_merge(cfg, legacy_overrides(cfg), true);
elseif ~strcmp(preset, 'corrected')
    error('study_config:preset', 'preset must be ''corrected'' or ''legacy''');
end
cfg.preset = preset;
cfg = drv_merge(cfg, user, true);
for i = 1:2:numel(args)
    if ~ischar(args{i})
        error('study_config:args', 'override names must be char');
    end
    cfg = drv_setpath(cfg, args{i}, args{i + 1});
end
if ischar(cfg.controllers)
    cfg.controllers = {cfg.controllers};
end
for i = 1:numel(cfg.controllers)
    cfg.controllers{i} = drv_ctrl_name(cfg.controllers{i});
end
if isempty(cfg.scenarios)
    cfg.scenarios = tuning_scenarios(cfg.P, cfg.scenario_set);
end
if isempty(cfg.rob.delay)
    cfg.rob.delay = getf(cfg.sim, 'delay', 0);
end
check(cfg);
end

% ---------------------------------------------------------------------------
function cfg = defaults()
P = boost_params();
cfg.preset = 'corrected';
cfg.P = P;
cfg.controllers = {'fcsmpc', 'cascade_pi', 'cascade_pid', 'smc', 'single_pi'};
cfg.vars.fcsmpc = mkvars({'lsw'}, 1e-3, 100, true);
cfg.vars.cascade_pi = mkvars({'Kp_i', 'Ki_i'}, [1e-3 1e-2], [0.5 1e3], [true true]);
cfg.vars.cascade_pid = mkvars({'Kp_i', 'Ki_i', 'Kd_i', 'Tf'}, ...
    [1e-3 1e-2 1e-9 2.5e-5], [0.5 1e3 1e-4 5e-3], [true true true true]);
cfg.vars.smc = mkvars({'K_s', 'phi'}, [0.01 0.01], [0.9 20], [false true]);
cfg.vars.single_pi = mkvars({'Kp_v', 'Ki_v'}, [1e-4 1e-2], [0.5 1e3], [true true]);
cfg.tune_outer = true;
cfg.outer_vars = mkvars({'KPV', 'KIV'}, [0.01 0.1], [5 1e4], [true true]);
cfg.objective_set = 'corrected';
cfg.mpc_fsw_objective = true;
cfg.extra_objectives = {};
cfg.sim = struct('ff_mode', 'measured', 'delay', 1, 'plant', 'avg');
cfg.scenario_set = 'nominal';
cfg.scenarios = [];
cfg.scenario_agg = 'max';
cfg.band_pct = 2;
cfg.penalty = struct('settling_ms', 2 * P.T_end * 1e3, 'recovery_ms', 2 * P.T_end * 1e3, ...
    'overshoot_pct', 200, 'peak_dev_V', 3 * P.Vref, 'fsw_kHz', P.fs / 1e3, ...
    'extra', 1e6, 'diverge_factor', 2);
cfg.seeds = 1:5;
cfg.mopso = struct('n_particles', 20, 'n_iter', 30, 'archive_size', 60, 'w', 0.6, ...
    'c1', 1.5, 'c2', 1.5, 'density', 'crowding', 'bound_handling', 'reset_velocity', ...
    'dedupe', true, 'compromise_method', 'ideal_point', 'legacy', false, 'verbose', false);
cfg.mopso_ctrl = struct();
cfg.init_baseline = true;
cfg.hv = struct('ref', 1.1, 'samples', 1e5, 'hist_samples', 1e4, 'seed', 1, 'history', true);
cfg.bound_tol_frac = 0.01;
cfg.backend = 'matlab';
cfg.batch_sim = true;
cfg.simcfg = struct();
cfg.out_file = '';
cfg.verbose = true;

rob.controllers = {};
rob.plant = 'switched';
rob.pwm = 'center';
rob.nsub = 50;
rob.ff_mode = 'measured';
rob.delay = [];
rob.extra_delay = 1;
rob.mc_seeds = 30;
rob.noise = struct('noise_std_i', 0.02, 'noise_std_v', 0.05, 'noise_std_io', 0.02, ...
    'noise_std_vin', 0.05, 'adc_bits', 10);
rob.parasitics = struct('rL', 0.05, 'rC', 0.02, 'Ron', 0.02, 'VF', 0.7);
rob.scenarios = [];
rob.scenario_names = {};
rob.keep_traces = {'nominal'};
rob.T_end = [];
rob.verbose = true;
cfg.rob = rob;

cfg.fig = struct('dir', 'figures', 'formats', {{'png'}}, 'font', 'Times New Roman', ...
    'font_size', 8, 'width_in', 3.5, 'height_in', 2.2, 'dpi', 600, ...
    'exclude', {{'single_pi'}}, 'dry_run', false, 'visible', 'off');
end

function o = legacy_overrides(cfg)
% notebook cell 3: bounds, budgets, seed 42, legacy objectives and MOPSO rules
o.vars.fcsmpc = mkvars({'lsw'}, 0.01, 2.0, false);
o.vars.cascade_pi = mkvars({'Kp_i', 'Ki_i'}, [0.005 1.0], [0.15 80.0], [false false]);
o.vars.cascade_pid = mkvars({'Kp_i', 'Ki_i', 'Kd_i', 'Tf'}, [0.005 1.0 1e-7 5e-5], ...
    [0.15 80.0 5e-5 5e-4], [false false false false]);
o.vars.smc = mkvars({'K_s', 'phi'}, [0.05 0.05], [0.80 2.0], [false false]);
o.vars.single_pi = mkvars({'Kp_v', 'Ki_v'}, [0.005 0.5], [0.20 30.0], [false false]);
o.tune_outer = false;
o.objective_set = 'legacy';
o.mpc_fsw_objective = false;
o.extra_objectives = {};
o.sim = struct('ff_mode', 'oracle', 'delay', 0, 'plant', 'avg');
o.seeds = 42;
o.mopso = struct('legacy', true, 'n_particles', 20, 'n_iter', 30);
o.mopso_ctrl = struct('fcsmpc', struct('n_particles', 15, 'n_iter', 25));
o.init_baseline = false;
o.rob = struct('ff_mode', 'oracle', 'plant', 'avg', 'delay', 0);
end

function v = mkvars(names, lb, ub, logm)
v = struct('names', {names}, 'lb', lb, 'ub', ub, 'log_scale', logical(logm));
end

function S = tuning_scenarios(P, set_name)
nom = mkscen('nominal', struct('kind', 'step', 'R0', P.R0, 'R_new', P.R1, 't_step', P.t_step), ...
    struct('kind', 'const', 'Vin', P.Vin), P.t_step, struct());
switch lower(set_name)
    case 'nominal'
        S = nom;
    case 'worst_case'
        S = nom;
        S(2) = mkscen('load_m55', struct('kind', 'step', 'R0', P.R0, 'R_new', 8.9, ...
            't_step', P.t_step), struct('kind', 'const', 'Vin', P.Vin), P.t_step, struct());
        S(3) = mkscen('load_release', struct('kind', 'step', 'R0', P.R1, 'R_new', P.R0, ...
            't_step', P.t_step), struct('kind', 'const', 'Vin', P.Vin), P.t_step, struct());
        S(4) = mkscen('vin_m20', struct('kind', 'const', 'R0', P.R0), ...
            struct('kind', 'step', 'Vin', P.Vin, 'pct', -0.20, 't_step', 0.060), 0.060, struct());
        Pp = P;
        Pp.L = 0.8 * P.L;
        S(5) = mkscen('L_m20', nom.load, nom.vin, P.t_step, struct('Pplant', Pp));
    otherwise
        error('study_config:scenario_set', 'scenario_set must be ''nominal'' or ''worst_case''');
end
end

function s = mkscen(name, lspec, vspec, t_evt, sim)
s = struct('name', name, 'load', lspec, 'vin', vspec, 't_evt', t_evt, 'sim', sim);
end

function check(cfg)
for i = 1:numel(cfg.controllers)
    c = cfg.controllers{i};
    if ~isfield(cfg.vars, c)
        error('study_config:vars', 'no decision variables for %s', c);
    end
    v = cfg.vars.(c);
    [~, ~, ng] = drv_ctrl_name(c);
    if numel(v.lb) ~= ng || numel(v.ub) ~= ng || numel(v.names) ~= ng || numel(v.log_scale) ~= ng
        error('study_config:vars', '%s needs %d decision variables', c, ng);
    end
    if any(v.ub < v.lb) || any(v.log_scale & v.lb <= 0)
        error('study_config:vars', 'invalid bounds for %s', c);
    end
end
if ~any(strcmp(cfg.objective_set, {'corrected', 'legacy'}))
    error('study_config:objective_set', 'objective_set must be ''corrected'' or ''legacy''');
end
if ~any(strcmp(cfg.backend, {'matlab', 'simulink'}))
    error('study_config:backend', 'backend must be ''matlab'' or ''simulink''');
end
if ~strcmp(cfg.scenario_agg, 'max')
    error('study_config:scenario_agg', 'only scenario_agg = ''max'' is implemented');
end
end

function v = getf(s, name, default)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
