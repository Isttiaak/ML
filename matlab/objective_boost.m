function [F, aux] = objective_boost(X, ctrl, P, cfg)
%OBJECTIVE_BOOST  Vectorised MOPSO objective for one controller (minimise).
%   F = objective_boost(X, ctrl, P, cfg)
%   [F, aux] = objective_boost(X, ctrl, P, cfg)
%   X    n x ndim decision rows (layout of drv_search_space: controller gains,
%        then KPV, KIV when cfg.tune_outer); a single row is fine.
%   ctrl controller name; P boost_params() ([] = cfg.P); cfg study_config.
%   F    n x nobj, always finite (MOPSO never sees NaN).
%
%   objective_set 'legacy' (notebook cell 1 compute_metrics, via
%   metrics_legacy on Vo):
%     [settling_ms overshoot_pct ss_error_V load_reg_V]
%     notebook semantics are kept (settling = t_evt sentinel when never
%     settled, 40-50 ms 'SS' window); only non-finite values are replaced by
%     the penalty row [t_evt_ms, penalty.overshoot_pct, penalty.peak_dev_V,
%     penalty.peak_dev_V].
%
%   objective_set 'corrected' (default; metrics_event, band cfg.band_pct):
%     1 startup_settling_ms    start-up settling into the +/-2 % band
%     2 startup_overshoot_pct  start-up overshoot
%     3 event_peak_dev_V       max |Vo - Vref| after the scenario event
%     4 recovery_ms            time to re-enter and stay in the band
%     5 fsw_kHz                FCS-MPC only, when cfg.mpc_fsw_objective:
%                              rising gate edges per second over the run / 1e3
%     + cfg.extra_objectives   optional metrics_event fields appended last,
%                              e.g. {'event_iae_Vms'} or {'post_offset_V'}
%                              (NaN or diverged -> penalty.extra = 1e6)
%   Penalties (cfg.penalty, all finite):
%     never settled before the event (NaN)  -> settling_ms   (200 = 2*T_end)
%     not back in band at the end (NaN)     -> recovery_ms   (200)
%     diverged run: any non-finite Vo, max|Vo| > diverge_factor*Vref (2*Vref)
%     or a simulation error -> the whole row is
%        [settling_ms overshoot_pct peak_dev_V recovery_ms fsw_kHz]
%        = [200 200 3*Vref 200 fs/1e3], which every non-diverged run
%        dominates (finite settling/recovery < T_end, overshoot <= 100 %,
%        peak deviation <= 3*Vref, fsw <= fs/2).
%     Any other NaN (should not occur) is replaced by the same row values.
%   With the switched plant the per-period envelope (Vo_min/Vo_max) is used
%   for overshoot and peak deviation.
%
%   Several cfg.scenarios: every objective is the maximum over the scenarios
%   (worst case).  Each scenario sets load, vin, its own event time t_evt
%   and extra sim options (e.g. Pplant) on top of cfg.sim.
%
%   aux: gains G, KPV, KIV, flags (n x nscen: 0 ok, 1 penalised NaN metric,
%   2 diverged/failed), Fs (n x nobj x nscen before aggregation), obj_names.

if nargin < 4 || isempty(cfg)
    cfg = study_config();
end
if nargin < 3 || isempty(P)
    P = cfg.P;
end
name = drv_ctrl_name(ctrl);
[G, KPV, KIV] = drv_decode(X, name, cfg);
n = size(X, 1);
legacy = strcmp(cfg.objective_set, 'legacy');
names = drv_objective_names(name, cfg);
nobj = numel(names);
pen = cfg.penalty;
scen = cfg.scenarios;
ns = numel(scen);
Fs = zeros(n, nobj, ns);
flags = zeros(n, ns);

for s = 1:ns
    opts = drv_merge(cfg.sim, scen(s).sim);
    opts.load = scen(s).load;
    opts.vin = scen(s).vin;
    opts.KPV = KPV;
    opts.KIV = KIV;
    outs = drv_run_batch(name, G, P, opts, cfg);
    t_evt = scen(s).t_evt;
    for i = 1:n
        [Fs(i, :, s), flags(i, s)] = one_run(outs(i), P, cfg, t_evt, legacy, names, pen);
    end
end
F = max(Fs, [], 3);
F = reshape(F, n, nobj);

if nargout > 1
    aux.G = G;
    aux.KPV = KPV;
    aux.KIV = KIV;
    aux.flags = flags;
    aux.Fs = Fs;
    aux.obj_names = names;
end
end

% ---------------------------------------------------------------------------
function [f, flag] = one_run(o, P, cfg, t_evt, legacy, names, pen)
Vref = P.Vref;
nobj = numel(names);
has_fsw = any(strcmp(names, 'fsw_kHz'));
if legacy
    bad_row = [t_evt * 1e3, pen.overshoot_pct, pen.peak_dev_V, pen.peak_dev_V];
else
    bad_row = [pen.settling_ms, pen.overshoot_pct, pen.peak_dev_V, pen.recovery_ms];
    if has_fsw
        bad_row(end + 1) = pen.fsw_kHz;
    end
    bad_row = [bad_row, repmat(getf(pen, 'extra', 1e6), 1, nobj - numel(bad_row))];
end
diverged = ~o.ok || isempty(o.Vo) || any(~isfinite(o.Vo));
if ~diverged && ~legacy
    diverged = max(abs(o.Vo)) > pen.diverge_factor * Vref;
end
if diverged
    f = bad_row;
    flag = 2;
    return;
end
flag = 0;
if legacy
    if abs(t_evt - P.t_step) < 1e-12
        m = metrics_legacy(o.Vo, P);
    else
        m = metrics_legacy(o.Vo, P, t_evt);
    end
    f = [m.settling_ms, m.overshoot_pct, m.ss_error_V, m.load_reg_V];
else
    ev = struct('t_evt', t_evt, 'band_pct', cfg.band_pct);
    if ~isempty(o.Vo_min) && ~isempty(o.Vo_max)
        ev.Vo_min = o.Vo_min;
        ev.Vo_max = o.Vo_max;
    end
    m = metrics_event(o.t, o.Vo, P, ev);
    f = [m.startup_settling_ms, m.startup_overshoot_pct, m.event_peak_dev_V, m.recovery_ms];
    if isnan(f(1))
        f(1) = pen.settling_ms;
        flag = 1;
    end
    if isnan(f(4))
        f(4) = pen.recovery_ms;
        flag = 1;
    end
    if has_fsw
        d = o.d;
        T = numel(d) * P.Ts;
        f(5) = sum(diff(d) > 0.5) / T / 1e3;
    end
    for j = numel(f) + 1:nobj
        f(j) = m.(names{j});
    end
end
nanj = ~isfinite(f);
if any(nanj)
    f(nanj) = bad_row(nanj);
    flag = max(flag, 1);
end
end

function v = getf(s, name, default)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
