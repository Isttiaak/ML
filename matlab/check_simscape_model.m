function R = check_simscape_model(copts)
%CHECK_SIMSCAPE_MODEL  Validate boost_simscape.slx against the exact switched plant.
%   R = check_simscape_model()          R = check_simscape_model(copts)
%   copts: folder (default: next to this file), rebuild (false).
%   Checks (ideal parts: diode Vf 0, switch Ron 1e-3 ohm):
%     (s1) open loop, d = 0.5, R = 20 ohm, started in steady state:
%          switching ripple from the fine log vs the analytic values
%          (dIL = Vin*d*Ts/L = 1.2 A, dVo = Io*d*Ts/C = 0.128 V) and the
%          period samples vs plant_switched_step (centre-aligned PWM)
%     (s2) closed loop, cascade_pi notebook gains, measured feed-forward,
%          one-sample delay, legacy load step: vs sim_boost(plant 'switched',
%          pwm 'center', delay 1).  Not bit-exact: PWM edges are quantised to
%          the solver step h and the diode / switch are not ideal.
%     (s3) the same with fcsmpc (gate applied directly through the carrier)
%   R: struct array name, pass, detail.

if nargin < 1 || isempty(copts)
    copts = struct();
end
here = fileparts(mfilename('fullpath'));
folder = getf(copts, 'folder', here);
mdl = 'boost_simscape';
P = boost_params();
if getf(copts, 'rebuild', false) || exist(fullfile(folder, [mdl '.slx']), 'file') ~= 2
    build_boost_simscape(folder);
end
if ~bdIsLoaded(mdl)
    load_system(fullfile(folder, [mdl '.slx']));
end
rows = cell(0, 3);

% ---- (s1) open loop
d = 0.5;
Rl = 20;
o = struct('open_loop', d, 'iL0', 4.8, 'Vo0', 48, 'T_end', 0.02, ...
    'load', struct('kind', 'const', 'R0', Rl, 'R', Rl));
S = setup_boost_simscape(mdl, 'cascade_pi', [], P, o);
tic;
so = sim(mdl);
t_ol = toc;
y = so.ylog;
fine = as_rows(so.yfine, 2);
tf = (0:size(fine, 1) - 1)' * S.h * S.fine_dec;
win = [0.019 0.020];
ppI = ripple_from_waveform(tf, fine(:, 1), P.fs, win);
[ppV, avgV] = ripple_from_waveform(tf, fine(:, 2), P.fs, win);
dI = P.Vin * d * P.Ts / P.L;
dV = (avgV / Rl) * d * P.Ts / P.C;
rows(end + 1, :) = {'(s1) open loop iL ripple vs Vin*d*Ts/L', abs(ppI - dI) / dI < 0.03, ...
    sprintf('%.4f A vs %.4f A (%.1f s sim)', ppI, dI, t_ol)};
rows(end + 1, :) = {'(s1) open loop Vo ripple vs Io*d*Ts/C', abs(ppV - dV) / dV < 0.08, ...
    sprintf('%.4f V vs %.4f V', ppV, dV)};
Pp = P;
Pp.pwm = 'center';
Pp.Ron = S.Ron_sw;
iL = S.iL0p;
vC = S.vC0p;
refI = zeros(S.N, 1);
refV = zeros(S.N, 1);
for kk = 1:S.N
    refI(kk) = iL;
    refV(kk) = vC;
    [iL, vC] = plant_switched_step(iL, vC, d, Rl, P.Vin, Pp, 50);
end
eI = max(abs(y(:, 2) - refI));
eV = max(abs(y(:, 1) - refV));
rows(end + 1, :) = {'(s1) open loop samples vs plant_switched_step', eI < 0.05 && eV < 0.05, ...
    sprintf('max|diL| %.3g A, max|dVo| %.3g V', eI, eV)};

% ---- (s2), (s3) closed loop
ref = jsondecode(fileread(fullfile(here, 'tests', 'ref', 'python_reference.json')));
cases = {'cascade_pi', ref.gains_mopso_python.cascade_pi(:)'; 'fcsmpc', P.gains_baseline.fcsmpc};
for c = 1:size(cases, 1)
    oc = struct('ff_mode', 'measured', 'delay', 1);
    S = setup_boost_simscape(mdl, cases{c, 1}, cases{c, 2}, P, oc);
    tic;
    so = sim(mdl);
    t_cl = toc;
    y = so.ylog;
    os = oc;
    os.plant = 'switched';
    os.pwm = 'center';
    os.nsub = 50;
    Pp = P;
    Pp.Ron = S.Ron_sw;
    os.Pplant = Pp;
    r = sim_boost(cases{c, 1}, cases{c, 2}, P, os);
    eV = max(abs(y(:, 1) - r.Vo));
    eI = max(abs(y(:, 2) - r.iL));
    k = round(0.040 / P.Ts):round(0.050 / P.Ts) - 1;
    mV = abs(mean(y(k, 1)) - mean(r.Vo(k)));
    if strcmp(cases{c, 1}, 'fcsmpc')
        tolV = 1.0;      % FCS-MPC switching patterns diverge after the first edge difference
    else
        tolV = 0.1;
    end
    rows(end + 1, :) = {sprintf('(s%d) closed loop %s vs sim_boost switched', c + 1, cases{c, 1}), ...
        mV < tolV && all(isfinite(y(:, 1))), ...
        sprintf('max|dVo| %.3g V, max|diL| %.3g A, |dmean Vo| 40-50 ms %.3g V (%.1f s sim)', ...
        eV, eI, mV, t_cl)};
end

R = struct('name', rows(:, 1), 'pass', rows(:, 2), 'detail', rows(:, 3));
fprintf('\nRES   CHECK                                                     DETAIL\n');
for i = 1:numel(R)
    fprintf('%-5s %-57s %s\n', ternary(R(i).pass, 'PASS', 'FAIL'), R(i).name, R(i).detail);
end
fprintf('%d of %d checks passed (boost_simscape)\n', sum([R.pass]), numel(R));
end

function X = as_rows(x, w)
x = squeeze(x);
if size(x, 2) ~= w && size(x, 1) == w
    x = x.';
end
X = x;
end

function v = getf(s, name, default)
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
