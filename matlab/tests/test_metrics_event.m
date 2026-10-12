function test_metrics_event()
%TEST_METRICS_EVENT  Synthetic signals with known answers for metrics_event.
%   Prints PASS/FAIL lines and errors at the end if any check failed.
P = boost_params();
Ts = P.Ts;
N = P.N;
t = (0:N - 1)' * Ts;
k = (1:N)';
nfail = 0;
tol = 1e-9;

% A: start-up with 5 % overshoot, settles at 12 ms with +0.5 V offset,
%    event dip to 45 V for 3 ms, back in band at 53 ms, ends at 48.2 V
Vo = 40 * ones(N, 1);
Vo(201:240) = 50.4;
Vo(241:1000) = 48.5;
Vo(1001:1060) = 45;
Vo(1061:1200) = 47.5;
Vo(1201:N) = 48.2;
m = metrics_event(t, Vo, P, struct('t_evt', 0.05));
nfail = nfail + chk('A startup_settling_ms = 12', m.startup_settling_ms, 12, tol);
nfail = nfail + chk('A startup_overshoot_pct = 5', m.startup_overshoot_pct, 5, tol);
nfail = nfail + chk('A pre_offset_V = 0.5', m.pre_offset_V, 0.5, tol);
nfail = nfail + chk('A pre_ripple_pp_V = 0', m.pre_ripple_pp_V, 0, tol);
nfail = nfail + chk('A event_peak_dev_V = 3', m.event_peak_dev_V, 3, tol);
nfail = nfail + chk('A event_peak_time_ms = 0', m.event_peak_time_ms, 0, tol);
nfail = nfail + chk('A event_undershoot_V = 3', m.event_undershoot_V, 3, tol);
nfail = nfail + chk('A event_overshoot_V = 0.2', m.event_overshoot_V, 0.2, tol);
nfail = nfail + chk('A event_iae_Vms = 20.5', m.event_iae_Vms, 20.5, tol);
nfail = nfail + chk('A recovery_ms = 3', m.recovery_ms, 3, tol);
nfail = nfail + chk('A post_offset_V = 0.2', m.post_offset_V, 0.2, tol);
nfail = nfail + chk('A settled_flag', double(m.settled_flag), 1, 0);
mw = metrics_event(t, Vo, P, struct('t_evt', 0.05, 'post_window', 2e-3));
nfail = nfail + chk('A post_window 2 ms -> not recovered (NaN)', double(isnan(mw.recovery_ms)), 1, 0);

% B: square wave +/-3 V, never settles
Vo = 48 + 3 * (-1) .^ floor((k - 1) / 20);
m = metrics_event(t, Vo, P, struct('t_evt', 0.05));
nfail = nfail + chk('B startup_settling_ms NaN (no sentinel)', double(isnan(m.startup_settling_ms)), 1, 0);
nfail = nfail + chk('B recovery_ms NaN', double(isnan(m.recovery_ms)), 1, 0);
nfail = nfail + chk('B settled_flag false', double(m.settled_flag), 0, 0);
nfail = nfail + chk('B pre_offset_V = 0.6 (signed mean)', m.pre_offset_V, 0.6, tol);
nfail = nfail + chk('B pre_ripple_pp_V = 6', m.pre_ripple_pp_V, 6, tol);

% C: always in band, small blip after the event
Vo = 48 * ones(N, 1);
Vo(1001:1010) = 48.5;
m = metrics_event(t, Vo, P, struct('t_evt', 0.05));
nfail = nfail + chk('C startup_settling_ms = 0', m.startup_settling_ms, 0, 0);
nfail = nfail + chk('C recovery_ms = 0 (never left band)', m.recovery_ms, 0, 0);
nfail = nfail + chk('C event_peak_dev_V = 0.5', m.event_peak_dev_V, 0.5, tol);

% D: per-period envelope (switched plant) gives the true ripple
Vo = 48 * ones(N, 1);
vmin = 47.9 * ones(N, 1);
vmax = 48.15 * ones(N, 1);
vmin(N) = NaN;
vmax(N) = NaN;
m = metrics_event(t, Vo, P, struct('t_evt', 0.05, 'Vo_min', vmin, 'Vo_max', vmax));
nfail = nfail + chk('D pre_ripple_pp_V = 0.25 (envelope)', m.pre_ripple_pp_V, 0.25, tol);
nfail = nfail + chk('D post_ripple_pp_V = 0.25 (envelope, NaN ignored)', m.post_ripple_pp_V, 0.25, tol);
nfail = nfail + chk('D event_peak_dev_V = 0.15', m.event_peak_dev_V, 0.15, tol);
nfail = nfail + chk('D ripple_from_envelope', double(m.ripple_from_envelope), 1, 0);
m0 = metrics_event(t, Vo, P, struct('t_evt', 0.05));
nfail = nfail + chk('D without envelope ripple = 0', m0.pre_ripple_pp_V, 0, 0);

% E: signed mean vs mean |e|
Vo = 48 * ones(N, 1);
Vo(901:2:1000) = 48.3;
Vo(902:2:1000) = 47.9;
m = metrics_event(t, Vo, P, struct('t_evt', 0.05));
nfail = nfail + chk('E pre_offset_V = 0.1 (not 0.2)', m.pre_offset_V, 0.1, tol);
nfail = nfail + chk('E pre_ripple_pp_V = 0.4', m.pre_ripple_pp_V, 0.4, tol);

% F: event after the end of the run
Vo = 48 * ones(N, 1);
m = metrics_event(t, Vo, P, struct('t_evt', 0.2));
nfail = nfail + chk('F event metrics NaN', double(isnan(m.event_peak_dev_V) && isnan(m.recovery_ms)), 1, 0);
nfail = nfail + chk('F startup settled at 0', m.startup_settling_ms, 0, 0);

% G: infeasible event (output ends far below the band)
Vo = 48 * ones(N, 1);
Vo(1001:N) = linspace(48, 38, N - 1000)';
m = metrics_event(t, Vo, P, struct('t_evt', 0.05));
nfail = nfail + chk('G recovery NaN when ending outside band', double(isnan(m.recovery_ms)), 1, 0);
nfail = nfail + chk('G event_undershoot_V = 10', m.event_undershoot_V, 10, tol);

% H: consistency with the legacy settling definition on real runs
J = core_read_json_exact(fullfile(fileparts(mfilename('fullpath')), 'ref', 'python_reference.json'));
o = sim_boost('cascade_pi', J.gains_mopso_python.cascade_pi, P);
m = metrics_event(o.t, o.Vo, P, struct('t_evt', P.t_step));
ml = metrics_legacy(o.Vo, P);
nfail = nfail + chk('H cascade_pi opt: startup settling = legacy 7.25 ms', m.startup_settling_ms, ml.settling_ms, 1e-9);
o = sim_boost('fcsmpc', [], P);
m = metrics_event(o.t, o.Vo, P, struct('t_evt', P.t_step));
ml = metrics_legacy(o.Vo, P);
nfail = nfail + chk(sprintf('H fcsmpc base: legacy sentinel %.2f ms -> NaN', ml.settling_ms), ...
    double(isnan(m.startup_settling_ms)), 1, 0);

if nfail > 0
    error('test_metrics_event:failed', '%d check(s) FAILED', nfail);
end
fprintf('test_metrics_event: all checks PASSED\n');
end

function f = chk(name, got, want, tol)
ok = abs(got - want) <= tol || (isnan(got) && isnan(want));
if ok
    fprintf('PASS  %s  (got %.12g)\n', name, got);
    f = 0;
else
    fprintf('FAIL  %s  (got %.12g, want %.12g)\n', name, got, want);
    f = 1;
end
end
