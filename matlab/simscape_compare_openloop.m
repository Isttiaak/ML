function res = simscape_compare_openloop(mdl_simscape, d_seq, opts)
%SIMSCAPE_COMPARE_OPENLOOP  Open-loop check of a hand-built Simscape boost against plant_switched_step.
%   res = simscape_compare_openloop(mdl_simscape, d_seq, opts)
%     mdl_simscape  name of the Simscape model ('' with reference_only or sim_data)
%     d_seq         N x 1 duty per PWM period, period k = [(k-1)*Ts, k*Ts)
%                   ([] = 0.5 for 0.02 s)
%     opts          optional struct:
%       P           plant parameters for the reference: L C Ts rL rC Ron VF and
%                   optional sync (default boost_params(), ideal)
%       pwm         'trailing' | 'center' (default P.pwm if present, else
%                   'trailing'); must match the PWM Generator of the model
%                   (Up counter = trailing edge, Up-down = centre aligned)
%       R, Vin      load and input voltage, scalar or N x 1 per period
%                   (default P.R0, P.Vin).  The Simscape model must use the
%                   same values; only the duty is fed from here.
%       iL0, vC0    initial inductor current and capacitor voltage (0, P.Vin);
%                   set the same Initial Targets (priority High) in Simscape
%       nsub        reference sample points per period (default 200)
%       reference_only  true: only compute the reference (no Simulink; runs
%                   in Octave)
%       sim_data    struct with column vectors t, iL, Vo to compare instead of
%                   running the model (e.g. traces exported from another run;
%                   works in Octave)
%       input       'inport' (default): duty -> root Inport 1 of the model
%                   (sample time Ts, so it is held per period);
%                   'variable': global variable opts.duty_var = [t d] for a
%                   From Workspace block (clear 'Interpolate data')
%       duty_var    'd_in'
%       log_iL, log_Vo  names of the logged inductor current and output voltage
%                   (To Workspace variables or logged signal names in logsout;
%                   default 'iL_sim', 'Vo_sim'); Timeseries, Structure With
%                   Time, Dataset element or Array (+ tout) are accepted
%       tol_iL, tol_V   PASS thresholds [A], [V] (default 0.05, 0.05)
%       verbose     true
%   res fields:
%     ref     reference at the period boundaries tb = (0:N)'*Ts: tb, iL, vC,
%             Vout (terminal voltage incl. ESR, left limit; Vout(1) = NaN) and
%             per period k = 1..N: iL_min iL_max iL_avg Vo_min Vo_max Vo_avg dcm
%     sim     the same quantities from the Simscape traces (empty when
%             reference_only): boundary values = left limits (first sample of
%             repeated time stamps), min/max over [tb(k), tb(k+1)], averages
%             from the cumulative trapezoid integral
%     err     max |sim - ref| per quantity and the period where it occurs
%     pass    all errors below tol_iL / tol_V
%     table   cell {quantity, max err, period, tol, pass}
%   Expected agreement: the reference is exact (closed-form solution of each
%   topology, see plant_switched_step); Simscape errors come from its solver
%   (fixed-step local solver: choose a sample time << Ts, e.g. Ts/200) and
%   the logging resolution (min/max).  Use rC = 0 first: with ESR the
%   terminal voltage jumps at every switching instant and a logger may
%   record either side of the jump at the period boundary.
%   Simulink part UNTESTED here (no MATLAB); reference_only / sim_data are
%   tested in Octave.

if nargin < 1
    mdl_simscape = '';
end
if nargin < 3 || isempty(opts)
    opts = struct();
end
P = getopt(opts, 'P', []);
if isempty(P)
    P = boost_params();
end
Ts = P.Ts;
if nargin < 2 || isempty(d_seq)
    d_seq = 0.5 * ones(fix(0.02 / Ts), 1);
end
d = double(d_seq(:));
N = numel(d);
pwm = getopt(opts, 'pwm', getopt(P, 'pwm', 'trailing'));
if ~ischar(pwm)
    if pwm ~= 0
        pwm = 'center';
    else
        pwm = 'trailing';
    end
end
pwm = lower(pwm);
if strcmp(pwm, 'centre')
    pwm = 'center';
end
if ~any(strcmp(pwm, {'trailing', 'center'}))
    error('simscape_compare_openloop:pwm', 'pwm must be ''trailing'' or ''center''');
end
Rk = per_period(getopt(opts, 'R', P.R0), N, 'R');
Vk = per_period(getopt(opts, 'Vin', P.Vin), N, 'Vin');
nsub = getopt(opts, 'nsub', 200);
verbose = getopt(opts, 'verbose', true);
tol_iL = getopt(opts, 'tol_iL', 0.05);
tol_V = getopt(opts, 'tol_V', 0.05);

% ---- reference: one plant_switched_step per period
Pq = P;
Pq.pwm = pwm;
ref.tb = (0:N)' * Ts;
ref.iL = zeros(N + 1, 1);
ref.vC = zeros(N + 1, 1);
ref.Vout = nan(N + 1, 1);
ref.iL(1) = getopt(opts, 'iL0', 0);
ref.vC(1) = getopt(opts, 'vC0', P.Vin);
fl = {'iL_min', 'iL_max', 'iL_avg', 'Vo_min', 'Vo_max', 'Vo_avg'};
for j = 1:numel(fl)
    ref.(fl{j}) = zeros(N, 1);
end
ref.dcm = false(N, 1);
for k = 1:N
    [ref.iL(k + 1), ref.vC(k + 1), pk] = plant_switched_step(ref.iL(k), ref.vC(k), d(k), Rk(k), Vk(k), ...
        Pq, nsub);
    ref.Vout(k + 1) = pk.Vout_end;
    ref.iL_min(k) = pk.iL_min;
    ref.iL_max(k) = pk.iL_max;
    ref.iL_avg(k) = pk.iL_avg;
    ref.Vo_min(k) = pk.Vout_min;
    ref.Vo_max(k) = pk.Vout_max;
    ref.Vo_avg(k) = pk.Vout_avg;
    ref.dcm(k) = pk.dcm;
end
res.ref = ref;
res.sim = [];
res.err = [];
res.pass = [];
res.table = {};
res.info = struct('N', N, 'Ts', Ts, 'pwm', pwm, 'nsub', nsub, 'model', mdl_simscape);
if verbose
    fprintf('reference: %d periods, %s PWM, final iL = %.4f A, Vout = %.4f V, DCM periods %d\n', ...
        N, pwm, ref.iL(end), ref.Vout(end), sum(ref.dcm));
end
if getopt(opts, 'reference_only', false)
    return;
end

% ---- traces: given, or from the Simscape model
sd = getopt(opts, 'sim_data', []);
if isempty(sd)
    sd = run_model(mdl_simscape, d, Ts, opts);
end
ts = double(sd.t(:));
xi = double(sd.iL(:));
xv = double(sd.Vo(:));
if numel(xi) ~= numel(ts) || numel(xv) ~= numel(ts)
    error('simscape_compare_openloop:trace', 'sim_data.t, iL and Vo must have the same length');
end

% ---- per-period quantities from the traces
tt = 1e-9 * Ts;
% snap time stamps within tt of a period boundary onto it (ulp jitter of
% k*Ts) and keep the logged order of repeated stamps (stable sort)
kb = round(ts / Ts);
near = abs(ts - kb * Ts) <= tt;
ts(near) = kb(near) * Ts;
if any(diff(ts) < 0)
    warning('simscape_compare_openloop:order', 'trace time stamps are not monotonic; sorting them');
    [ts, ord] = sort(ts);
    xi = xi(ord);
    xv = xv(ord);
end
s.tb = ref.tb;
s.iL = left_value(ts, xi, ref.tb, tt);
s.Vout = left_value(ts, xv, ref.tb, tt);
s.Vout(1) = NaN;
s.iL_avg = period_avg(ts, xi, ref.tb, tt);
s.Vo_avg = period_avg(ts, xv, ref.tb, tt);
for j = [1 2 4 5]
    s.(fl{j}) = nan(N, 1);
end
[i0, i1] = period_ranges(ts, ref.tb, tt);
for k = 1:N
    if i0(k) <= i1(k)
        m = i0(k):i1(k);
        s.iL_min(k) = min(xi(m));
        s.iL_max(k) = max(xi(m));
        s.Vo_min(k) = min(xv(m));
        s.Vo_max(k) = max(xv(m));
    end
end
res.sim = s;

% ---- errors
q = {'iL at period start', 'iL', tol_iL; 'Vout at period end', 'Vout', tol_V; ...
    'iL min', 'iL_min', tol_iL; 'iL max', 'iL_max', tol_iL; 'iL avg', 'iL_avg', tol_iL; ...
    'Vout min', 'Vo_min', tol_V; 'Vout max', 'Vo_max', tol_V; 'Vout avg', 'Vo_avg', tol_V};
tab = cell(size(q, 1), 5);
allok = true;
for j = 1:size(q, 1)
    e = abs(s.(q{j, 2}) - ref.(q{j, 2}));
    e(isnan(ref.(q{j, 2}))) = 0;            % undefined reference (Vout at t = 0)
    nn = isnan(e);
    if any(nn)
        emax = Inf;
        kmax = find(nn, 1);
    elseif isempty(e)
        emax = 0;
        kmax = 0;
    else
        [emax, kmax] = max(e);
    end
    okj = emax <= q{j, 3};
    allok = allok && okj;
    res.err.(q{j, 2}) = emax;
    res.err.([q{j, 2} '_at']) = kmax;
    tab(j, :) = {q{j, 1}, emax, kmax, q{j, 3}, okj};
end
res.pass = allok;
res.table = tab;
if verbose
    fprintf('%-20s %12s %8s %8s  %s\n', 'quantity', 'max|err|', 'index', 'tol', 'result');
    for j = 1:size(tab, 1)
        fprintf('%-20s %12.4g %8d %8.3g  %s\n', tab{j, 1}, tab{j, 2}, tab{j, 3}, tab{j, 4}, ...
            ternary(tab{j, 5}, 'PASS', 'FAIL'));
    end
    fprintf('(index: boundary k = 1..N+1 at t = (k-1)*Ts for iL/Vout, else period k)\n');
    if allok
        fprintf('Simscape model agrees with plant_switched_step within the tolerances.\n');
    else
        fprintf(['Disagreement: check L, C, rL, rC, Ron, VF, R, Vin, initial targets, the PWM carrier\n' ...
            '(Up = trailing, Up-down = centre), the solver step (<< Ts) and the logged signals.\n']);
    end
end
end

% ========================================================================
function sd = run_model(mdl, d, Ts, opts)
% Simulink / Simscape side (UNTESTED here)
if isempty(mdl)
    error('simscape_compare_openloop:model', 'give the Simscape model name, opts.sim_data or opts.reference_only');
end
if isempty(which('sim')) || isempty(which('load_system'))
    error('simscape_compare_openloop:simulink', 'Simulink is required (or use opts.sim_data / reference_only)');
end
if ~bdIsLoaded(mdl)
    load_system(mdl);
end
N = numel(d);
td = [(0:N - 1)' * Ts; N * Ts];
dd = [d; d(end)];
in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('StopTime', sprintf('%.17g', N * Ts));
if strcmpi(getopt(opts, 'input', 'inport'), 'inport')
    in = in.setExternalInput(timeseries(dd, td));
else
    in = in.setVariable(getopt(opts, 'duty_var', 'd_in'), [td dd]);
end
try
    so = sim(in);
catch err
    error('simscape_compare_openloop:sim', 'simulation failed: %s', err.message);
end
em = '';
try
    em = char(so.ErrorMessage);
catch
end
if ~isempty(em)
    error('simscape_compare_openloop:sim', 'simulation failed: %s', em);
end
[t1, x1] = get_trace(so, getopt(opts, 'log_iL', 'iL_sim'));
[t2, x2] = get_trace(so, getopt(opts, 'log_Vo', 'Vo_sim'));
sd.t = t1;
sd.iL = x1;
if numel(t2) == numel(t1) && all(t2 == t1)
    sd.Vo = x2;
else
    sd.Vo = interp1(unique_first(t2), x2(first_idx(t2)), t1, 'linear');
end
end

function [t, x] = get_trace(so, name)
v = [];
try
    v = so.get(name);
catch
end
if isempty(v)
    try
        lo = so.get('logsout');
        v = lo.get(name);
    catch
    end
end
if isempty(v)
    error('simscape_compare_openloop:log', ['logged signal ''%s'' not found: add a To Workspace block ' ...
        '(Timeseries) with that variable name after a PS-Simulink Converter'], name);
end
if isa(v, 'Simulink.SimulationData.Signal')
    v = v.Values;
end
if isa(v, 'timeseries')
    t = v.Time;
    x = v.Data;
elseif isstruct(v) && isfield(v, 'time') && isfield(v, 'signals')
    t = v.time;
    x = v.signals.values;
else
    t = so.get('tout');
    x = v;
end
t = double(t(:));
x = double(squeeze(x));
x = x(:);
if numel(x) ~= numel(t)
    error('simscape_compare_openloop:log', 'signal ''%s'' must be scalar (got %d values for %d times)', ...
        name, numel(x), numel(t));
end
end

function [i0, i1] = period_ranges(ts, tb, tt)
% sample range of each period [tb(k), tb(k+1)] in the sorted trace: at a
% boundary with repeated time stamps the last sample (right limit) opens
% period k and the first one (left limit) closes period k-1
M = numel(ts);
nb = numel(tb);
fge = zeros(nb, 1);             % first index with ts >= tb - tt
lle = zeros(nb, 1);             % last index with ts <= tb + tt
j = 1;
for b = 1:nb
    while j <= M && ts(j) < tb(b) - tt
        j = j + 1;
    end
    fge(b) = j;
end
j = M;
for b = nb:-1:1
    while j >= 1 && ts(j) > tb(b) + tt
        j = j - 1;
    end
    lle(b) = j;
end
at = fge <= lle;                % samples exist at this boundary
a0 = at(1:nb - 1);
i0 = fge(1:nb - 1);
l0 = lle(1:nb - 1);
i0(a0) = l0(a0);
a1 = at(2:nb);
i1 = lle(2:nb);
f1 = fge(2:nb);
i1(a1) = f1(a1);
end

function v = left_value(ts, x, tb, tt)
% value at tb (first sample of repeated time stamps = left limit at a jump)
[tu, iu] = unique(ts, 'first');
xu = x(iu);
tq = min(max(tb, tu(1)), tu(end));
v = interp1(tu, xu, tq, 'linear');
v(tb < tu(1) - tt | tb > tu(end) + tt) = NaN;
end

function a = period_avg(ts, x, tb, tt)
% exact average of the piecewise-linear trace over each [tb(k), tb(k+1)]
I = [0; cumsum(diff(ts) .* (x(1:end - 1) + x(2:end)) / 2)];
[tu, iu] = unique(ts, 'first');
Iu = I(iu);
tq = min(max(tb, tu(1)), tu(end));
Ib = interp1(tu, Iu, tq, 'linear');
Ib(tb < tu(1) - tt | tb > tu(end) + tt) = NaN;
a = diff(Ib) ./ diff(tb);
end

function tu = unique_first(t)
tu = unique(t, 'first');
end

function i = first_idx(t)
[~, i] = unique(t, 'first');
end

function v = per_period(v, N, name)
if isscalar(v)
    v = v * ones(N, 1);
elseif numel(v) == N
    v = double(v(:));
else
    error('simscape_compare_openloop:size', '%s must be scalar or have one value per period (%d)', name, N);
end
end

function v = getopt(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end

function r = ternary(c, a, b)
if c
    r = a;
else
    r = b;
end
end
