function test_core_equivalence()
%TEST_CORE_EQUIVALENCE  MATLAB/Octave port vs the Python notebook reference.
%   Compares sim_boost/metrics_legacy with tests/ref/*.json/*.csv, checks the
%   numeric helpers, and smoke-tests the new (non-legacy) options.
%   Prints PASS/FAIL lines and errors at the end if anything failed.
here = fileparts(mfilename('fullpath'));
refdir = fullfile(here, 'ref');
P = boost_params();
J = core_read_json_exact(fullfile(refdir, 'python_reference.json'));
[h, M] = core_read_csv(fullfile(refdir, 'python_traces.csv'));
[hr, MR] = core_read_csv(fullfile(refdir, 'python_robustness_traces.csv'));
TOL = 1e-9;
nf = 0;

%% 0. parameters
f = fieldnames(J.params);
bad = {};
for i = 1:numel(f)
    if ~isequal(P.(f{i}), J.params.(f{i}))
        bad{end + 1} = f{i}; %#ok<AGROW>
    end
end
nf = nf + report('boost_params == python params (all 17 fields, bit-exact)', isempty(bad), strjoin(bad, ' '));
nf = nf + report('time vector t = (0:N-1)*Ts == python t_s', ...
    isequal((0:P.N - 1)' * P.Ts, M(:, 1)), '');

%% 1. numeric helpers
nf = nf + report('round_half_even ties to even', isequal(round_half_even([0.5 1.5 2.5 -0.5 -1.5 -2.5 2.4 2.6 -2.6 3]), ...
    [0 2 2 0 -2 -2 2 3 -3 3]), '');
nf = nf + report('py_round = Python round(float,n)', isequal(py_round([2.675 0.125 0.375 1.0005 70.40015], 2), ...
    [2.67 0.12 0.38 1.0 70.4]) && py_round(70.40015, 4) == 70.4001 && py_round(26.35, 3) == 26.35, '');
nf = nf + report('core_np_round = numpy round(np.float64,n)', core_np_round(70.40015, 4) == 70.4002 && ...
    core_np_round(2.675, 2) == 2.68, '');
x = 1 ./ (1:999)';
s_seq = 0;
for i = 1:numel(x)
    s_seq = s_seq + x(i);
end
nf = nf + report('core_np_sum = numpy pairwise sum (differs from sequential)', ...
    core_np_sum(x) == hex2num('401df0192117b700') && s_seq == hex2num('401df0192117b6fd'), '');
cs = J.sense_quantisation_cases;
nbad = 0;
for i = 1:numel(cs)
    y = sense_adc(cs(i).x, 0, cs(i).bits, cs(i).lo, cs(i).hi);
    nbad = nbad + (y ~= cs(i).y);
end
nf = nf + report(sprintf('sense_adc quantisation cases (%d, bit-exact)', numel(cs)), nbad == 0, sprintf('%d mismatches', nbad));
nf = nf + report('sense_adc bits=0 is pass-through + noise', sense_adc(3.3, 0.1, 0, 0, 12) == 3.3 + 0.1, '');

%% 2. legacy trajectories and metrics (cell 1 sim_* + compute_metrics)
ctrls = {'fcsmpc', 'cascade_pi', 'cascade_pid', 'smc', 'single_pi'};
sets = {'base', 'opt'};
gsets = {'gains_baseline', 'gains_mopso_python'};
for s = 1:2
    for c = 1:numel(ctrls)
        cn = ctrls{c};
        g = J.(gsets{s}).(cn);
        o = sim_boost(cn, g, P);
        pre = [cn '_' sets{s} '_'];
        dI = max(abs(o.iL - col(h, M, [pre 'iL'])));
        dV = max(abs(o.Vo - col(h, M, [pre 'Vo'])));
        dU = 0;
        if strcmp(cn, 'fcsmpc')
            u = col(h, M, [pre 'u']);
            dU = max(abs(o.u(1:end - 1) - u(1:end - 1)));
        end
        dmax = max([dI dV dU]);
        nf = nf + report(sprintf('trace %-11s %-4s max|diff| iL=%.3g Vo=%.3g u=%.3g%s', cn, sets{s}, dI, dV, dU, ...
            exact_tag(dmax)), dmax <= TOL, '');
        mref = metric_vec(J.metrics.(sets{s}).(cn));
        mm = metric_vec(metrics_legacy(o.Vo, P));
        mpy = metric_vec(metrics_legacy(col(h, M, [pre 'Vo']), P));
        nf = nf + report(sprintf('metrics %-11s %-4s == python (sim)   [%s]', cn, sets{s}, num2str(mm, '%g ')), ...
            isequal(mm, mref), ['ref: ' num2str(mref, '%g ')]);
        nf = nf + report(sprintf('metrics %-11s %-4s == python (on python Vo trace)', cn, sets{s}), ...
            isequal(mpy, mref), ['got: ' num2str(mpy, '%g ')]);
    end
end

%% 3. robustness battery (cell 12 simulate_controller + metrics_scenario)
scen = {'load_m25', 'load_m50', 'load_m75', 'load_ramp', 'vin_p20'};
for i = 1:numel(scen)
    [opts, t_evt] = scenario_opts(scen{i});
    for c = 2:4
        cn = ctrls{c};
        o = sim_boost(cn, J.gains_mopso_python.(cn), P, opts);
        pre = [scen{i} '_' cn '_'];
        dV = max(abs(o.Vo - col(hr, MR, [pre 'Vo'])));
        dI = max(abs(o.iL - col(hr, MR, [pre 'iL'])));
        dmax = max(dV, dI);
        nf = nf + report(sprintf('robust %-9s %-11s max|diff| Vo=%.3g iL=%.3g%s', scen{i}, cn, dV, dI, exact_tag(dmax)), ...
            dmax <= TOL, '');
        mref = metric_vec(J.robustness_metrics.(scen{i}).(cn));
        mm = metric_vec(metrics_legacy(o.Vo, P, t_evt));
        mpy = metric_vec(metrics_legacy(col(hr, MR, [pre 'Vo']), P, t_evt));
        nf = nf + report(sprintf('robust metrics %-9s %-11s == python (sim and python trace)', scen{i}, cn), ...
            isequal(mm, mref) && isequal(mpy, mref), ['sim: ' num2str(mm, '%g ') ' ref: ' num2str(mref, '%g ')]);
    end
end

%% 4. new options (smoke tests)
g = J.gains_mopso_python.cascade_pi;
o0 = sim_boost('cascade_pi', g, P);
o = sim_boost('cascade_pi', g, P, struct('KPV', P.KPV, 'KIV', P.KIV, 'ff_mode', 'oracle', ...
    'delay', 0, 'plant', 'avg', 'adc_bits', 0));
ok_default = isequal(o.Vo, o0.Vo);
nf = nf + report('explicit legacy options == defaults', ok_default, '');

o = sim_boost('cascade_pi', g, P, struct('ff_mode', 'measured', 'T_end', 0.3));
e_end = abs(o.Vo(end) - P.Vref);
nf = nf + report(sprintf('ff_mode measured (T_end 0.3): |Vo(end)-Vref| = %.2e V, IL_ff(end) = %.4f A', e_end, o.IL_ff(end)), ...
    all(isfinite(o.Vo)) && e_end < 0.02 && abs(o.IL_ff(end) - P.IL_ff1) < 0.02 && ~isequal(o.Vo(1:P.N), o0.Vo), '');

gb = P.gains_baseline.cascade_pi;
o = sim_boost('cascade_pi', gb, P, struct('ff_mode', 'none', 'T_end', 0.5));
e_end = abs(o.Vo(end) - P.Vref);
nf = nf + report(sprintf('ff_mode none (baseline gains, T_end 0.5): IL_ff = 0, |Vo(end)-Vref| = %.2e V', e_end), ...
    all(o.IL_ff == 0) && e_end < 0.01, '');
o = sim_boost('cascade_pi', gb, P, struct('T_end', 0.3));
nf = nf + report('T_end 0.3 -> N = fix(0.3/Ts) = 5999 (Python int() semantics)', numel(o.t) == 5999, '');

o = sim_boost('cascade_pi', g, P, struct('KPV', 0.3, 'KIV', 30));
nf = nf + report('outer gains override KPV/KIV changes response', ~isequal(o.Vo, o0.Vo) && all(isfinite(o.Vo)), '');

o = sim_boost('cascade_pi', g, P, struct('delay', 1));
nf = nf + report('delay=1: d(1) = D_ss and d(k+1) = d_cmd(k)', o.d(1) == P.D_ss && ...
    isequal(o.d(2:end), o.d_cmd(1:end - 1)) && all(isfinite(o.Vo)), '');
o = sim_boost('fcsmpc', [], P, struct('delay', 1));
nf = nf + report('delay=1 FCS-MPC: u(1) = 0 and u(k+1) = u_cmd(k)', o.d(1) == 0 && ...
    isequal(o.d(2:end), o.d_cmd(1:end - 1)), '');

nopt = struct('noise_std_i', 0.02, 'noise_std_v', 0.05, 'adc_bits', 10, ...
    'iL_adc_range', P.iL_adc_range_legacy, 'seed', 1);
rng(123, 'twister');
r_before = rand;
rng(123, 'twister');
o1 = sim_boost('cascade_pi', g, P, nopt);
r_after = rand;
o2 = sim_boost('cascade_pi', g, P, nopt);
nopt.seed = 2;
o3 = sim_boost('cascade_pi', g, P, nopt);
q = o1.Vo_meas / (60 / 1023);
nf = nf + report('noise: same seed -> identical runs', isequal(o1.Vo, o2.Vo) && isequal(o1.iL, o2.iL), '');
nf = nf + report('noise: different seed -> different runs', ~isequal(o1.Vo, o3.Vo), '');
nf = nf + report('noise: caller RNG state restored', r_before == r_after, '');
nf = nf + report('noise+ADC: Vo_meas on the 10-bit grid of [0 60] V', max(abs(q - round(q))) < 1e-9, '');

t75 = struct('load', struct('kind', 'step', 'pct', 0.75, 't_step', 0.05), 'T_end', 0.3);
t55 = struct('load', struct('kind', 'step', 'pct', 0.55, 't_step', 0.05), 'T_end', 0.3);
o75 = sim_boost('cascade_pi', gb, P, t75);
o55 = sim_boost('cascade_pi', gb, P, t55);
late = o75.t >= 0.15;
V_sat = sqrt(P.Vin * P.IL_MAX * 5);          % power-limited output with iL = IL_MAX
nf = nf + report(sprintf('-75%% step (R=5 ohm, needs 19.2 A > IL_MAX): Vo(end) = %.3f V ~ sqrt(Vin*IL_MAX*R) = %.3f V', ...
    o75.Vo(end), V_sat), max(o75.Vo(late)) < P.Vref * 0.98 && all(o75.IL_ref(late) == P.IL_MAX) && ...
    abs(o75.Vo(end) - V_sat) < 0.05, '');
nf = nf + report(sprintf('-55%% step (R=9 ohm, needs 10.67 A): Vo(end) = %.4f V, IL_ref(end) = %.3f A', ...
    o55.Vo(end), o55.IL_ref(end)), abs(o55.Vo(end) - P.Vref) < 0.01 && o55.IL_ref(end) < P.IL_MAX, '');

Pp = P;
Pp.L = 1.2 * P.L;
Pp.C = 0.8 * P.C;
o = sim_boost('cascade_pid', J.gains_mopso_python.cascade_pid, P, struct('Pplant', Pp));
on = sim_boost('cascade_pid', J.gains_mopso_python.cascade_pid, P);
nf = nf + report('Pplant L/C mismatch runs and differs from nominal', all(isfinite(o.Vo)) && ...
    ~isequal(o.Vo, on.Vo), '');

o = sim_boost('fcsmpc', J.gains_mopso_python.fcsmpc, P, struct('ff_mode', 'measured', 'vin', ...
    struct('kind', 'step', 'pct', 0.2, 't_step', 0.06)));
nf = nf + report('FCS-MPC with measured ff and Vin step runs', all(isfinite(o.Vo)) && all(o.d == 0 | o.d == 1), '');

if exist('plant_switched_step', 'file') == 2
    try
        o = sim_boost('cascade_pi', g, P, struct('plant', 'switched', 'nsub', 50));
        okf = all(isfinite(o.Vo)) && all(o.Vo_max(1:end - 1) >= o.Vo_min(1:end - 1)) && ...
            abs(mean(o.Vo(o.t >= 0.045 & o.t < 0.05)) - P.Vref) < 1.0;
        nf = nf + report(sprintf('switched plant smoke test (mean Vo 45-50 ms = %.3f V, pp ripple %.3f V)', ...
            mean(o.Vo(o.t >= 0.045 & o.t < 0.05)), max(o.Vo_max(900:999)) - min(o.Vo_min(900:999))), okf, '');
    catch err
        nf = nf + report('switched plant smoke test', false, err.message);
    end
else
    fprintf('SKIP  switched plant smoke test (plant_switched_step.m not present yet)\n');
end

cg = {'ctrl_outer_pi', 'ctrl_inner_pi', 'ctrl_inner_pid', 'ctrl_smc', 'ctrl_single_pi', 'ctrl_fcsmpc', ...
    'plant_avg_step', 'sense_adc', 'round_half_even'};
nocg = {};
for i = 1:numel(cg)
    if isempty(strfind(fileread(which(cg{i})), '%#codegen'))
        nocg{end + 1} = cg{i}; %#ok<AGROW>
    end
end
nf = nf + report('Simulink block functions carry %#codegen', isempty(nocg), strjoin(nocg, ' '));

if nf > 0
    error('test_core_equivalence:failed', '%d check(s) FAILED', nf);
end
fprintf('test_core_equivalence: all checks PASSED\n');
end

function [opts, t_evt] = scenario_opts(name)
t_evt = 0.05;
switch name
    case 'load_m25'
        opts.load = struct('kind', 'step', 'pct', 0.25, 't_step', 0.050);
    case 'load_m50'
        opts.load = struct('kind', 'step', 'pct', 0.50, 't_step', 0.050);
    case 'load_m75'
        opts.load = struct('kind', 'step', 'pct', 0.75, 't_step', 0.050);
    case 'load_ramp'
        opts.load = struct('kind', 'ramp', 't0', 0.050, 't1', 0.060, 'R_end', 10.0);
    case 'vin_p20'
        opts.load = struct('kind', 'step', 'pct', 0.0, 't_step', 999);
        opts.vin = struct('kind', 'step', 'pct', 0.20, 't_step', 0.060);
        t_evt = 0.06;
end
end

function v = col(h, M, name)
j = find(strcmp(h, name));
if numel(j) ~= 1
    error('test_core_equivalence:col', 'column %s not found', name);
end
v = M(:, j);
end

function v = metric_vec(m)
% values in the legacy order Settling Overshoot SSError Ripple LoadReg
v = cell2mat(struct2cell(m))';
end

function s = exact_tag(d)
if d == 0
    s = '  (bit-exact)';
else
    s = '';
end
end

function f = report(name, ok, detail)
if ok
    fprintf('PASS  %s\n', name);
    f = 0;
else
    fprintf('FAIL  %s  %s\n', name, detail);
    f = 1;
end
end
