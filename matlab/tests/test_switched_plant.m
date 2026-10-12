function res = test_switched_plant(csv_dir)
%TEST_SWITCHED_PLANT  Tests of plant_switched_step (and sw_boost_period).
%   Trailing-edge PWM (the default); centre-aligned PWM (P.pwm = 'center')
%   is tested in tests/test_centre_pwm.m.
%   res = test_switched_plant()         run all checks, print a table
%   res = test_switched_plant(csv_dir)  also write the CSV files read by
%                                       python3 -I tests/switched_crosscheck.py csv_dir
%                                       (trailing and centre-PWM cases)
%   res(i) has fields name, passed, detail.  Raises an error at the end if
%   any check failed.  Takes about 30 s in Octave 8.4.
%   a  open-loop steady state vs textbook CCM ripple formulas (R = 20, 10)
%   b  ESR rC = 20 mOhm: Vout pk-pk grows by ~ rC*iL_valley (see below)
%   c  light load: DCM, iL >= 0, Vout vs analytic DCM gain, re-conduction
%   d  period averages vs the averaged model of plant_avg_step
%   e  nsub = 50 vs 400
%   f  scalar code-generation kernel sw_boost_period == plant_switched_step
%   g  closed-form 2x2 exponential vs expm; h  edge cases;  i  sim_boost
if nargin < 1
    csv_dir = '';
end
P = boost_params();
Ts = P.Ts; L = P.L; C = P.C; Vin = P.Vin;
res = struct('name', {}, 'passed', {}, 'detail', {});

% ---------------------------------------------------------------- a ------
d = 0.5;
pp0 = 0; ss20 = [];
for R = [20 10]
    Vexp = Vin / (1 - d);
    Iexp = Vexp^2 / (Vin * R);
    [info, dx] = sw_settle(P, Iexp, Vexp, d, R, Vin, 50, 4000);
    dIexp = Vin * d * Ts / L;
    dVexp = (Vexp / R) * d * Ts / C;
    dI = info.iL_max - info.iL_min;
    dV = info.Vout_max - info.Vout_min;
    tag = sprintf('a_ss_R%d', R);
    res = sw_add(res, [tag '_Vout_avg'], abs(info.Vout_avg / Vexp - 1) < 0.005, ...
        sprintf('Vout_avg %.4f V vs %.2f (tol 0.5%%), cyclic |dx| %.1e', info.Vout_avg, Vexp, dx));
    res = sw_add(res, [tag '_iL_avg'], abs(info.iL_avg / Iexp - 1) < 0.005, ...
        sprintf('iL_avg %.4f A vs %.2f (tol 0.5%%)', info.iL_avg, Iexp));
    res = sw_add(res, [tag '_iL_ripple'], abs(dI / dIexp - 1) < 0.02, ...
        sprintf('iL pk-pk %.4f A vs Vin*d*Ts/L = %.4f (tol 2%%)', dI, dIexp));
    res = sw_add(res, [tag '_Vout_ripple'], abs(dV / dVexp - 1) < 0.05, ...
        sprintf('Vout pk-pk %.4f V vs Io*d*Ts/C = %.4f (tol 5%%)', dV, dVexp));
    if R == 20
        pp0 = dV;
        ss20 = info;
    end
end
% centre-of-ON sample = period average in CCM; the period-start sample is the valley
res = sw_add(res, 'a_mid_on_sample', abs(ss20.iL_mid_on - ss20.iL_avg) < 2e-3 && ...
    abs((ss20.iL_avg - ss20.iL_min) - 0.6) < 0.01, ...
    sprintf('iL_mid_on %.4f A vs iL_avg %.4f A; valley (t=0) %.4f A = avg - dI/2', ...
    ss20.iL_mid_on, ss20.iL_avg, ss20.iL_min));

% ---------------------------------------------------------------- b ------
% ESR: Vout = kR*(vC + rC*iD).  The ESR term steps up by kR*rC*iL(t_on) at
% turn-off and down by kR*rC*iL(Ts) at turn-on.  vC is lowest at t_on and
% highest at Ts, so Vout_max = vC_max + rC*(iL_valley - Io) (end of OFF) and
% Vout_min = vC_min - rC*Io (end of ON): pk-pk grows by ~ rC*iL_valley,
% bounded above by rC*iL_peak (the turn-off step).
PE = P; PE.rC = 20e-3; R = 20;
[infE, ~, iLE, vCE] = sw_settle(PE, 4.8, 48, d, R, Vin, 50, 4000);
ppE = infE.Vout_max - infE.Vout_min;
dpp = ppE - pp0;
pred = PE.rC * infE.iL_min;
res = sw_add(res, 'b_esr_ripple_increase', abs(dpp / pred - 1) < 0.10 && dpp > 0 && ...
    dpp < PE.rC * infE.iL_max, sprintf(['Vout pk-pk %.4f V (rC=0: %.4f), increase %.4f V vs ' ...
    'rC*iL_valley %.4f (tol 10%%), < rC*iL_peak %.4f'], ppE, pp0, dpp, pred, PE.rC * infE.iL_max));
kR = R / (R + PE.rC);
off = kR * PE.rC * (iLE - vCE / R);
res = sw_add(res, 'b_esr_terminal_offset', abs((infE.Vout_end - vCE) - off) < 1e-9 && off > 0, ...
    sprintf('Vout_end - vC = %.6f V = kR*rC*(iL - vC/R) > 0 (sampled at end of OFF)', infE.Vout_end - vCE));
res = sw_add(res, 'b_esr_dc_unchanged', abs(infE.Vout_avg / ss20.Vout_avg - 1) < 0.005 && ...
    abs(infE.Vout_avg - infE.vC_avg) < 2e-3, sprintf(['Vout_avg %.4f V (rC=0: %.4f), ' ...
    'Vout_avg - vC_avg = %.1e V (mean iC ~ 0)'], infE.Vout_avg, ss20.Vout_avg, infE.Vout_avg - infE.vC_avg));

% ---------------------------------------------------------------- c ------
R = 500; d = 0.5;
K = 2 * L / (R * Ts);
Mdcm = (1 + sqrt(1 + 4 * d^2 / K)) / 2;
Vdcm = Vin * Mdcm;
iL = 0; vC = 70; imin = Inf;
for k = 1:10000
    [iL, vC, info] = plant_switched_step(iL, vC, d, R, Vin, P, 50);
    imin = min([imin, info.iL_min, iL]);
end
d2 = d * Vin / (info.Vout_avg - Vin);
tid = (1 - d - d2) * Ts;
res = sw_add(res, 'c_dcm_flag_and_iL_nonneg', info.dcm && imin >= 0, ...
    sprintf('dcm=%d, min iL over 10000 periods = %.3g A', info.dcm, imin));
res = sw_add(res, 'c_dcm_Vout', info.Vout_avg > Vin / (1 - d) && abs(info.Vout_avg / Vdcm - 1) < 0.01, ...
    sprintf('Vout_avg %.3f V > CCM 48 V; analytic DCM %.3f V (tol 1%%)', info.Vout_avg, Vdcm));
res = sw_add(res, 'c_dcm_idle_time', abs(info.t_idle / tid - 1) < 0.02, ...
    sprintf('t_idle %.3f us vs (1-d-d2)*Ts = %.3f us (tol 2%%)', info.t_idle * 1e6, tid * 1e6));
iL = 0; vC = Vin; imin = Inf; ndcm = 0;
for k = 1:2000
    [iL, vC, info] = plant_switched_step(iL, vC, d, R, Vin, P, 50);
    imin = min([imin, info.iL_min, iL]);
    ndcm = ndcm + info.dcm;
end
res = sw_add(res, 'c_dcm_startup_iL_nonneg', imin >= 0 && ndcm > 0, ...
    sprintf('startup R=500 from Vo=Vin: %d DCM periods of 2000, min iL %.3g A', ndcm, imin));
[iL1, ~, info] = plant_switched_step(0, 24.05, 0, 1, Vin, P, 50);
tre = 1 * C * log(24.05 / Vin);
res = sw_add(res, 'c_reconduction', abs(info.t_idle / tre - 1) < 1e-9 && iL1 > 0 && info.dcm, ...
    sprintf('idle until Vout = Vin: t_idle %.6f us vs R*C*ln(24.05/24) = %.6f us, iL1 %.4f A', ...
    info.t_idle * 1e6, tre * 1e6, iL1));

% ---------------------------------------------------------------- d ------
N = 400; M = 200;
% d1: diode plant, CCM transient (duty 0.5->0.55 and R 20->10) starting
% from the cyclic steady state; averaged model starts from its period average
[ss, ~, iL0, vC0] = sw_settle(P, 4.8, 48, 0.5, 20, Vin, 50, 6000);
[Is, Vs, nd] = sw_run(P, iL0, vC0, 0.55, 10, Vin, 50, N);
[Ia, Va] = sw_avg_fine(P, ss.iL_avg, ss.vC_avg, 0.55, 10, Vin, M, N);
eI = max(abs(Is - Ia)); eV = max(abs(Vs - Va));
res = sw_add(res, 'd1_avg_model_ccm_step', eI < 0.02 && eV < 0.02 && nd == 0, ...
    sprintf(['diode, CCM step over 20 ms: max|diL_avg| %.4f A, max|dVout_avg| %.4f V ' ...
    '(tol 0.02; iL %.1f..%.1f A)'], eI, eV, min(Is), max(Is)));
% d2: synchronous plant, startup from iL = 0, Vo = Vin (no DCM possible)
PS = P; PS.sync = true;
[Is, Vs] = sw_run(PS, 0, Vin, 0.5, 20, Vin, 50, N);
[Ia, Va, iamin] = sw_avg_fine(P, 0, Vin, 0.5, 20, Vin, M, N);
eI = max(abs(Is - Ia)); eV = max(abs(Vs - Va));
res = sw_add(res, 'd2_avg_model_sync_startup', eI < 0.5 && eV < 0.5, ...
    sprintf(['sync, startup 20 ms: max|diL_avg| %.3f A, max|dVout_avg| %.3f V (tol 0.5; ' ...
    'swing iL %.1f..%.1f A, Vout %.1f..%.1f V)'], eI, eV, min(Is), max(Is), min(Vs), max(Vs)));
% d3: diode plant, same startup: enters DCM where the averaged model has iL < 0
[Isd, Vsd, nd] = sw_run(P, 0, Vin, 0.5, 20, Vin, 50, N);
res = sw_add(res, 'd3_diode_startup_departs', nd > 0 && iamin < 0, ...
    sprintf(['diode startup: %d DCM periods; averaged model reaches iL = %.1f A, so it ' ...
    'departs by up to %.1f V (legacy averaged model = synchronous boost)'], nd, iamin, ...
    max(abs(Vsd - Va))));
il = zeros(N + 1, 1); vl = il; vl(1) = Vin;
for k = 1:N
    [il(k + 1), vl(k + 1)] = plant_avg_step(il(k), vl(k), 0.5, 20, Vin, L, C, Ts);
end
Vl = 0.5 * (vl(1:N) + vl(2:N + 1));
Il = 0.5 * (il(1:N) + il(2:N + 1));
fprintf(['INFO d: legacy Euler at Ts vs accurate averaged model, open-loop startup: ' ...
    'max|diL| %.2f A, max|dVo| %.2f V (Euler halves the LC damping at Ts = 50 us)\n'], ...
    max(abs(Il - Ia)), max(abs(Vl - Va)));
fprintf('INFO d: switched sync plant vs legacy Euler at Ts: max|dVo| %.2f V\n', max(abs(Vs - Vl)));

% ---------------------------------------------------------------- e ------
PP = P; PP.rL = 0.05; PP.rC = 0.02; PP.Ron = 0.03; PP.VF = 0.7;
cases = {P, PP};
for c = 1:2
    Pc = cases{c};
    A = sw_runall(Pc, 0, Vin, 0.5, 20, Vin, 50, N);
    B = sw_runall(Pc, 0, Vin, 0.5, 20, Vin, 400, N);
    es = max(max(abs(A(:, 1:3) - B(:, 1:3))));
    ea = max(max(abs(A(:, 4:5) - B(:, 4:5))));
    em = max(max(abs(A(:, 6:9) - B(:, 6:9))));
    ef = max(abs(A(:, 10) - B(:, 10)));
    res = sw_add(res, sprintf('e_nsub_50_vs_400_case%d', c), es < 1e-9 && ea < 1e-3 && em < 2e-3 && ef == 0, ...
        sprintf(['startup 400 periods (%d DCM): states %.1e (tol 1e-9), averages %.1e ' ...
        '(tol 1e-3), min/max %.1e (tol 2e-3)'], sum(A(:, 10)), es, ea, em));
end

% ---------------------------------------------------------------- f ------
u = sw_lcg(3000, 777);
emax = 0;
for j = 1:300
    r = u(10 * j - 9:10 * j);
    Pj = P;
    if r(8) < 0.5
        Pj.rL = 0.1 * r(1); Pj.rC = 0.05 * r(2); Pj.Ron = 0.05 * r(3); Pj.VF = r(4);
    end
    Pj.sync = r(9) < 0.2;
    dj = r(5);
    if r(10) < 0.1, dj = 0; elseif r(10) > 0.9, dj = 1; end
    Rj = 2 + 998 * r(6)^3;
    ns = [1 2 7 50];
    ns = ns(1 + floor(4 * r(7)));
    iL0 = 15 * r(1) * r(3) - 0.5 * Pj.sync;
    vC0 = 10 + 70 * r(2);
    emax = max(emax, sw_cmp_kernel(Pj, iL0, vC0, dj, Rj, 10 + 30 * r(4), ns));
end
res = sw_add(res, 'f_kernel_single_period', emax < 1e-9, ...
    sprintf('300 random periods (parasitics, sync, d=0/1, nsub 1..50): max rel diff %.1e', emax));
emax = 0;
iL = 0; vC = Vin; ik = 0; vk = Vin;
for k = 1:400
    [iL, vC] = plant_switched_step(iL, vC, 0.5, 20, Vin, PP, 50);
    [ik, vk] = sw_boost_period(ik, vk, 0.5, 20, Vin, L, C, Ts, PP.rL, PP.rC, PP.Ron, PP.VF, 50, 0);
    emax = max([emax, abs(iL - ik), abs(vC - vk)]);
end
res = sw_add(res, 'f_kernel_400_periods', emax < 1e-9, ...
    sprintf('diode startup with parasitics, 400 periods: max |diff| %.1e', emax));

% ---------------------------------------------------------------- g ------
gcase = {[0.05 0.02 0 0.7], 20, 10, 40, 0; [1 0 0 0], 0.1, 50, 10, 0; [0.05 0.02 0.03 0], 20, 2, 40, 1};
emax = 0;
for c = 1:size(gcase, 1)
    pr = gcase{c, 1};
    Pg = P; Pg.rL = pr(1); Pg.rC = pr(2); Pg.Ron = pr(3); Pg.VF = pr(4);
    R = gcase{c, 2}; iL0 = gcase{c, 3}; vC0 = gcase{c, 4}; dg = gcase{c, 5};
    [i1, v1, info] = plant_switched_step(iL0, vC0, dg, R, Vin, Pg, 10);
    kR = R / (R + Pg.rC); g = 1 / (R + Pg.rC);
    if dg == 0
        A = [-(Pg.rL + kR * Pg.rC) / L, -kR / L; kR / C, -g / C];
        b = [(Vin - Pg.VF) / L; 0];
    else
        A = [-(Pg.rL + Pg.Ron) / L, 0; 0, -g / C];
        b = [Vin / L; 0];
    end
    E = expm([A, b; 0 0 0] * Ts);
    x = E * [iL0; vC0; 1];
    emax = max([emax, abs(i1 - x(1)) / max(1, abs(x(1))), abs(v1 - x(2)) / max(1, abs(x(2)))]);
    if c == 2
        m11 = 0.5 * (A(1, 1) - A(2, 2));
        Dl = m11^2 + A(1, 2) * A(2, 1);
    end
    if info.dcm
        emax = Inf;
    end
end
res = sw_add(res, 'g_closed_form_vs_expm', emax < 1e-10 && Dl > 0, ...
    sprintf('OFF underdamped, OFF overdamped (Dl=%.2g>0), ON with rL+Ron: max rel diff %.1e', Dl, emax));

% ---------------------------------------------------------------- h ------
ok = true; msg = '';
[i1, v1, info] = plant_switched_step(3, 40, 1, 20, Vin, P, 20);
ok = ok && abs(i1 - (3 + Vin * Ts / L)) < 1e-12 && abs(v1 - 40 * exp(-Ts / (20 * C))) < 1e-12 ...
    && info.t_on == Ts && abs(info.Vout_end - v1) < 1e-12;
msg = [msg sprintf('d=1 exact ramp %d; ', ok)];
[i2, v2] = plant_switched_step(3, 40, 1.3, 20, Vin, P, 20);
ok = ok && i2 == i1 && v2 == v1;
[~, ~, info] = plant_switched_step(3, 40, 0, 20, Vin, P, 20);
ok = ok && info.t_on == 0 && info.iL_max == 3;
[a1, b1] = plant_switched_step(-1, 40, 0.3, 20, Vin, P, 20);
[a2, b2] = plant_switched_step(0, 40, 0.3, 20, Vin, P, 20);
ok = ok && a1 == a2 && b1 == b2;
[a3] = plant_switched_step(-1, 40, 0.3, 20, Vin, PS, 20);
ok = ok && a3 < a2;
msg = [msg sprintf('clamp/sync %d; ', ok)];
[a1, b1] = plant_switched_step(4, 45, 0.37, 20, Vin, PP, 1);
[a2, b2] = plant_switched_step(4, 45, 0.37, 20, Vin, PP, 100);
ok = ok && abs(a1 - a2) < 1e-10 && abs(b1 - b2) < 1e-10;
msg = [msg sprintf('nsub=1 exact %d; ', ok)];
[a1, b1, info] = plant_switched_step(4, 45, 0.37, Inf, Vin, PP, 20);
ok = ok && all(isfinite([a1, b1, info.Vout_end, info.Vout_avg, info.iL_avg]));
[a1, b1, info] = plant_switched_step(4, 45, 1e-12, 20, Vin, P, 20);
ok = ok && all(isfinite([a1, b1, info.Vout_avg]));
msg = [msg sprintf('R=Inf, d=1e-12 finite %d', ok)];
res = sw_add(res, 'h_edge_cases', ok, msg);

% ---------------------------------------------------------------- i ------
% Closed loop through sim_boost (core agent's file, plant = 'switched').
% With trailing-edge PWM sim_boost samples iL at the period start = ripple
% valley, so the inner loop sees ~dI/2 = 0.6 A less than the average
% current; this check documents that bias (opts.pwm = 'center' samples the
% middle of the ON pulse and removes it, see test_centre_pwm).
try
    o = sim_boost('cascade_pi', [], P, struct('plant', 'switched', 'nsub', 50));
    oa = sim_boost('cascade_pi', [], P, struct('plant', 'avg'));
    w = (o.t >= 0.040) & (o.t < 0.050);
    w2 = (o.t >= 0.090) & (o.t < 0.0999);
    rip = mean(o.Vo_max(w) - o.Vo_min(w));
    ripI = mean(o.iL_max(w) - o.iL_min(w));
    bias = mean(o.iL_avg(w) - o.iL(w));
    ok = abs(rip / ((48 / 20) * 0.5 * Ts / C) - 1) < 0.10 && abs(ripI / 1.2 - 1) < 0.10 && ...
        ~any(o.dcm(w)) && abs(bias / (ripI / 2) - 1) < 0.10 && abs(mean(o.Vo(w2)) - 48) < 1;
    msg = sprintf(['cascade_pi closed loop, 40-50 ms: Vo pk-pk %.4f V, iL pk-pk %.3f A, ' ...
        'iL_avg - iL(sampled) %.3f A; mean Vo 40-50 ms %.3f V (avg plant %.3f), 90-100 ms %.3f V (avg plant %.3f)'], ...
        rip, ripI, bias, mean(o.Vo(w)), mean(oa.Vo(w)), mean(o.Vo(w2)), mean(oa.Vo(w2)));
catch err
    ok = false;
    msg = ['sim_boost call failed: ' err.message];
end
res = sw_add(res, 'i_sim_boost_switched', ok, msg);

% ---------------------------------------------------------------- timing -
nt = 1000;
for ns = [50 200]
    iL = 4.8; vC = 48;
    tic;
    for k = 1:nt
        [iL, vC] = plant_switched_step(iL, vC, 0.5, 20, Vin, P, ns); %#ok<ASGLU>
    end
    t1 = toc / nt;
    iL = 4.8; vC = 48;
    tic;
    for k = 1:nt
        [iL, vC] = sw_boost_period(iL, vC, 0.5, 20, Vin, L, C, Ts, 0, 0, 0, 0, ns, 0); %#ok<ASGLU>
    end
    t2 = toc / nt;
    fprintf('INFO timing nsub=%d: plant_switched_step %.0f us/call, sw_boost_period %.0f us/call\n', ...
        ns, t1 * 1e6, t2 * 1e6);
end

if ~isempty(csv_dir)
    files = sw_crosscheck_export(csv_dir);
    fprintf('INFO wrote %d cross-check CSV files to %s\n', numel(files), csv_dir);
end

fprintf('\n%-30s %-6s %s\n', 'check', 'result', 'detail');
for j = 1:numel(res)
    r = 'PASS';
    if ~res(j).passed
        r = 'FAIL';
    end
    fprintf('%-30s %-6s %s\n', res(j).name, r, res(j).detail);
end
nf = sum(~[res.passed]);
fprintf('%d checks, %d failed\n', numel(res), nf);
if nf > 0
    error('test_switched_plant:failed', '%d check(s) failed', nf);
end
end

% ======================================================================
function res = sw_add(res, name, passed, detail)
res(end + 1) = struct('name', name, 'passed', logical(passed), 'detail', detail);
end

function [info, dx, iL, vC] = sw_settle(P, iL, vC, d, R, Vin, nsub, n)
% run n periods from (iL, vC); returns the last period's info, the final
% state and dx = state change over one more period (cyclic steady state)
for k = 1:n
    [iL, vC, info] = plant_switched_step(iL, vC, d, R, Vin, P, nsub);
end
[i2, v2] = plant_switched_step(iL, vC, d, R, Vin, P, nsub);
dx = max(abs(i2 - iL), abs(v2 - vC));
end

function [Is, Vs, nd] = sw_run(P, iL, vC, d, R, Vin, nsub, n)
Is = zeros(n, 1); Vs = Is; nd = 0;
for k = 1:n
    [iL, vC, info] = plant_switched_step(iL, vC, d, R, Vin, P, nsub);
    Is(k) = info.iL_avg;
    Vs(k) = info.Vout_avg;
    nd = nd + info.dcm;
end
end

function A = sw_runall(P, iL, vC, d, R, Vin, nsub, n)
% per period: iL1 vC1 Vout_end iL_avg Vout_avg iL_min iL_max Vout_min Vout_max dcm
A = zeros(n, 10);
for k = 1:n
    [iL, vC, info] = plant_switched_step(iL, vC, d, R, Vin, P, nsub);
    A(k, :) = [iL, vC, info.Vout_end, info.iL_avg, info.Vout_avg, info.iL_min, info.iL_max, ...
        info.Vout_min, info.Vout_max, info.dcm];
end
end

function [Ia, Va, imin] = sw_avg_fine(P, ia, va, d, R, Vin, M, n)
% averaged model (plant_avg_step equations) with M Euler sub-steps per Ts,
% returned as per-period trapezoid averages; imin = lowest iL reached
h = P.Ts / M;
Ia = zeros(n, 1); Va = Ia; imin = Inf;
for k = 1:n
    si = 0.5 * ia; sv = 0.5 * va;
    for j = 1:M
        [ia, va] = plant_avg_step(ia, va, d, R, Vin, P.L, P.C, h);
        if j < M
            si = si + ia; sv = sv + va;
        end
        imin = min(imin, ia);
    end
    Ia(k) = (si + 0.5 * ia) / M;
    Va(k) = (sv + 0.5 * va) / M;
end
end

function e = sw_cmp_kernel(P, iL, vC, d, R, Vin, nsub)
% max relative difference between the vectorised and the scalar kernel
rL = 0; rC = 0; Ron = 0; VF = 0;
if isfield(P, 'rL'), rL = P.rL; rC = P.rC; Ron = P.Ron; VF = P.VF; end
[a1, a2, info] = plant_switched_step(iL, vC, d, R, Vin, P, nsub);
[b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13] = sw_boost_period(iL, vC, d, R, Vin, ...
    P.L, P.C, P.Ts, rL, rC, Ron, VF, nsub, double(P.sync));
x = [a1, a2, info.Vout_end, info.iL_min, info.iL_max, info.Vout_min, info.Vout_max, ...
    info.iL_avg, info.Vout_avg, double(info.dcm), info.t_idle * 1e6, info.iL_mid_on, info.Vout_mid_on];
y = [b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11 * 1e6, b12, b13];
e = max(abs(x - y) ./ max(1, abs(x)));
end

function u = sw_lcg(n, seed)
u = zeros(n, 1);
x = seed;
for j = 1:n
    x = mod(16807 * x, 2147483647);
    u(j) = x / 2147483647;
end
end
