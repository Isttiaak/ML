function res = test_centre_pwm(skip_core)
%TEST_CENTRE_PWM  Centre-aligned PWM in the switched plant + ripple_from_waveform.
%   res = test_centre_pwm()      run all checks (about 60 s in Octave 8.4,
%                                including test_core_equivalence)
%   res = test_centre_pwm(true)  skip the call to test_core_equivalence
%   Prints a table and raises an error at the end if any check failed.
%   a  open-loop CCM steady state (d = 0.5, R = 20): start sample = average,
%      ripple and mean equal to trailing edge
%   b  DCM (R = 500): iL >= 0, analytic DCM gain, same steady state as trailing
%   c  sw_boost_period == plant_switched_step in both PWM modes
%   d  nsub 50 vs 400;  e  one centre period vs expm; d = 0 / d = 1 limits
%   f  ripple_from_waveform on synthetic triangles (uniform / non-uniform)
%   g  ripple_from_waveform on plant_switched_step's waveform output
%   h  sim_boost closed loop (cascade_pi): sampling bias, Vo offset
%   i  legacy defaults unchanged (incl. test_core_equivalence)
if nargin < 1
    skip_core = false;
end
P = boost_params();
PC = P; PC.pwm = 'center';
Ts = P.Ts; L = P.L; C = P.C; Vin = P.Vin;
res = struct('name', {}, 'passed', {}, 'detail', {});

% ---------------------------------------------------------------- a ------
d = 0.5; R = 20;
[it, ~, ~, st] = cp_settle(P, 4.8, 48, d, R, Vin, 50, 4000);
[ic, ~, ~, sc] = cp_settle(PC, 4.8, 48, d, R, Vin, 50, 4000);
dIc = ic.iL_max - ic.iL_min; dVc = ic.Vout_max - ic.Vout_min;
dIt = it.iL_max - it.iL_min; dVt = it.Vout_max - it.Vout_min;
res = cp_add(res, 'a_start_sample_eq_avg', abs(sc(1) - ic.iL_avg) < 0.02 * dIc && ...
    abs((it.iL_avg - st(1)) / (dIt / 2) - 1) < 0.02, sprintf(['centre: iL(t=0) %.4f A, iL_avg %.4f A, ' ...
    'diff %.2f%% of ripple (tol 2%%); trailing: iL(t=0) %.4f A = avg - %.4f A (dI/2 = %.3f)'], ...
    sc(1), ic.iL_avg, 100 * (sc(1) - ic.iL_avg) / dIc, st(1), it.iL_avg - st(1), dIt / 2));
dIexp = Vin * d * Ts / L;
dVexp = (48 / R) * d * Ts / C;
res = cp_add(res, 'a_ripple_eq_trailing', abs(dIc - dIt) < 1e-6 && abs(dVc - dVt) < 1e-6 && ...
    abs(dIc / dIexp - 1) < 0.02 && abs(dVc / dVexp - 1) < 0.05, sprintf(['iL pk-pk %.5f A (trailing %.5f, ' ...
    'formula %.4f), Vout pk-pk %.5f V (trailing %.5f, formula %.4f)'], dIc, dIt, dIexp, dVc, dVt, dVexp));
res = cp_add(res, 'a_Vout_avg', abs(ic.Vout_avg / 48 - 1) < 0.005 && abs(ic.Vout_avg - it.Vout_avg) < 1e-3, ...
    sprintf('Vout_avg %.5f V (trailing %.5f V, tol 0.5%% of 48 V)', ic.Vout_avg, it.Vout_avg));
res = cp_add(res, 'a_sampled_Vout', abs(ic.Vout_end - ic.Vout_avg) < 0.1 * dVc && ...
    abs(it.Vout_end - it.Vout_max) < 0.1 * dVt, sprintf(['Vout at the sampling instant: centre %.4f V ' ...
    '(avg %.4f), trailing %.4f V (max %.4f, end of OFF)'], ic.Vout_end, ic.Vout_avg, it.Vout_end, it.Vout_max));
res = cp_add(res, 'a_info_fields', ic.iL_mid_on == sc(1) && ic.Vout_mid_on == sc(2) && ...
    ic.t_on == d * Ts && ~ic.dcm, sprintf('centre iL_mid_on = start sample %.4f A, t_on = %.1f us', ...
    ic.iL_mid_on, ic.t_on * 1e6));

% ---------------------------------------------------------------- b ------
R = 500;
K = 2 * L / (R * Ts);
Vdcm = Vin * (1 + sqrt(1 + 4 * d^2 / K)) / 2;
d2 = d * Vin / (Vdcm - Vin);
[ib, imin] = cp_settle(PC, 0, Vdcm, d, R, Vin, 50, 4000);
it = cp_settle(P, 0, Vdcm, d, R, Vin, 50, 4000);
tid = (1 - d - d2) * Ts;
res = cp_add(res, 'b_dcm_ss', ib.dcm && imin >= 0 && abs(ib.Vout_avg / Vdcm - 1) < 0.01 && ...
    abs(ib.t_idle / tid - 1) < 0.02 && abs(ib.Vout_avg - it.Vout_avg) < 0.01, sprintf(['dcm=%d, min iL %.3g A, ' ...
    'Vout_avg %.3f V (trailing %.3f, analytic %.3f), t_idle %.3f us (analytic %.3f); start sample %.3f A ' ...
    '(DCM: = peak/2, not the average %.3f A)'], ib.dcm, imin, ib.Vout_avg, it.Vout_avg, Vdcm, ib.t_idle * 1e6, ...
    tid * 1e6, ib.iL_mid_on, ib.iL_avg));
iL = 0; vC = Vin; imin = Inf; ndcm = 0;
for k = 1:2000
    [iL, vC, info] = plant_switched_step(iL, vC, d, R, Vin, PC, 50);
    imin = min([imin, info.iL_min, iL]);
    ndcm = ndcm + info.dcm;
end
res = cp_add(res, 'b_dcm_startup', imin >= 0 && ndcm > 1000 && all(isfinite([iL, vC])), ...
    sprintf('startup R=500 from Vo=Vin: %d DCM periods of 2000, min iL %.3g A, Vout %.2f V', ndcm, imin, vC));

% ---------------------------------------------------------------- c ------
u = cp_lcg(3000, 777);
emax = [0 0];
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
    for m = 0:1
        emax(m + 1) = max(emax(m + 1), cp_cmp_kernel(Pj, iL0, vC0, dj, Rj, 10 + 30 * r(4), ns, m));
    end
end
res = cp_add(res, 'c_kernel_random_both_modes', all(emax < 1e-11), ...
    sprintf('300 random periods each (parasitics, sync, d=0/1, nsub 1..50): max rel diff trailing %.1e, centre %.1e', ...
    emax(1), emax(2)));
PP = PC; PP.rL = 0.05; PP.rC = 0.02; PP.Ron = 0.03; PP.VF = 0.7;
e400 = 0; nd = 0;
iL = 0; vC = Vin; ik = 0; vk = Vin;
for k = 1:400
    dk = 0.5 + 0.3 * sin(2 * pi * k / 37);
    [iL, vC, info] = plant_switched_step(iL, vC, dk, 20 + 480 * (k > 200), Vin, PP, 50);
    [ik, vk] = sw_boost_period(ik, vk, dk, 20 + 480 * (k > 200), Vin, L, C, Ts, PP.rL, PP.rC, PP.Ron, ...
        PP.VF, 50, 0, 1);
    e400 = max([e400, abs(iL - ik), abs(vC - vk)]);
    nd = nd + info.dcm;
end
[x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13] = sw_boost_period(3, 40, 0.37, 20, Vin, L, C, Ts, ...
    0.05, 0.02, 0.03, 0.7, 20, 0);
[y1, y2, y3, y4, y5, y6, y7, y8, y9, y10, y11, y12, y13] = sw_boost_period(3, 40, 0.37, 20, Vin, L, C, Ts, ...
    0.05, 0.02, 0.03, 0.7, 20, 0, 0);
same14 = isequal([x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13], ...
    [y1, y2, y3, y4, y5, y6, y7, y8, y9, y10, y11, y12, y13]);
res = cp_add(res, 'c_kernel_400_periods_center', e400 < 1e-9 && nd > 0 && same14, sprintf(['centre, ' ...
    'parasitics, duty sweep, R 20 -> 500 (%d DCM periods): max |diff| %.1e (tol 1e-9); 14-input call == ' ...
    'pwm_center 0: %d'], nd, e400, same14));

% ---------------------------------------------------------------- d ------
PPc = PP;
cases = {PC, PPc};
for c = 1:2
    A = cp_runall(cases{c}, 0, Vin, 0.5, 20, Vin, 50, 400);
    B = cp_runall(cases{c}, 0, Vin, 0.5, 20, Vin, 400, 400);
    es = max(max(abs(A(:, 1:3) - B(:, 1:3))));
    ea = max(max(abs(A(:, 4:5) - B(:, 4:5))));
    em = max(max(abs(A(:, 6:9) - B(:, 6:9))));
    ef = max(abs(A(:, 10) - B(:, 10)));
    res = cp_add(res, sprintf('d_nsub_50_vs_400_case%d', c), es < 1e-9 && ea < 1e-3 && em < 2e-3 && ef == 0, ...
        sprintf(['centre, startup 400 periods (%d DCM): states %.1e (tol 1e-9), averages %.1e (tol 1e-3), ' ...
        'min/max %.1e (tol 2e-3)'], sum(A(:, 10)), es, ea, em));
end

% ---------------------------------------------------------------- e ------
Pg = PC; Pg.rL = 0.05; Pg.rC = 0.02; Pg.Ron = 0.03; Pg.VF = 0.7;
R = 20; dg = 0.37; x0 = [6; 45];
[i1, v1, info] = plant_switched_step(x0(1), x0(2), dg, R, Vin, Pg, 10);
kR = R / (R + Pg.rC); g = 1 / (R + Pg.rC);
Aon = [-(Pg.rL + Pg.Ron) / L, 0, Vin / L; 0, -g / C, 0; 0 0 0];
Aoff = [-(Pg.rL + kR * Pg.rC) / L, -kR / L, (Vin - Pg.VF) / L; kR / C, -g / C, 0; 0 0 0];
th = 0.5 * dg * Ts;
x = expm(Aon * th) * expm(Aoff * (Ts - dg * Ts)) * expm(Aon * th) * [x0; 1];
eg = max(abs([i1 - x(1), v1 - x(2)]) ./ max(1, abs(x(1:2)')));
res = cp_add(res, 'e_center_period_vs_expm', eg < 1e-10 && ~info.dcm && abs(info.Vout_end - kR * v1) < 1e-12, ...
    sprintf('ON(dTs/2) OFF ON(dTs/2) with parasitics: max rel diff %.1e, Vout_end = kR*vC', eg));
[a1, b1, ia] = plant_switched_step(0.3, 30, 0, 50, Vin, PP, 20);
Pt = PP; Pt.pwm = 'trailing';
[a2, b2, ib0] = plant_switched_step(0.3, 30, 0, 50, Vin, Pt, 20);
ok0 = a1 == a2 && b1 == b2 && isequal(ia, ib0);
[a1, b1, ia] = plant_switched_step(3, 40, 1, 20, Vin, PP, 20);
[a2, b2, ib1] = plant_switched_step(3, 40, 1, 20, Vin, Pt, 20);
ok1 = abs(a1 - a2) < 1e-12 && abs(b1 - b2) < 1e-12 && abs(ia.iL_avg - ib1.iL_avg) < 1e-12;
[a3, b3] = plant_switched_step(3, 40, 0.4, 20, Vin, setfield(PP, 'pwm', 'centre'), 20); %#ok<SFLD>
[a4, b4] = plant_switched_step(3, 40, 0.4, 20, Vin, setfield(PP, 'pwm', 1), 20); %#ok<SFLD>
[a5, b5] = plant_switched_step(3, 40, 0.4, 20, Vin, PP, 20);
ok2 = a3 == a5 && b3 == b5 && a4 == a5 && b4 == b5;
res = cp_add(res, 'e_limits_and_aliases', ok0 && ok1 && ok2, sprintf(['d=0 centre == trailing ' ...
    '(bit-exact) %d; d=1 centre == trailing (1e-12) %d; ''centre'' / 1 aliases %d'], ok0, ok1, ok2));

% ---------------------------------------------------------------- f ------
fs = P.fs; Tw = 1 / fs; A0 = 4.8; Bpp = 1.2; Dr = 0.4;
tri = @(tt) cp_tri(tt, Tw, Dr, A0, Bpp, 0.13 * Tw);
tv = cp_tri_vertices(12, Tw, Dr, 0.13 * Tw);
% f1 uniform grid that contains the vertices (200 points per period)
t1 = (0:12 * 200)' * (Tw / 200);
t1 = unique([t1; tv]);
[pp, av, inf1] = ripple_from_waveform(t1, tri(t1), fs, [2 * Tw, 12 * Tw]);
ok = abs(pp - Bpp) < 1e-12 && abs(av - A0) < 1e-12 && inf1.n_periods == 10 && ~inf1.aliasing;
res = cp_add(res, 'f_uniform_grid', ok, sprintf('pp %.15g (1.2), avg %.15g (4.8), %d periods, %d pts', ...
    pp, av, inf1.n_periods, inf1.n_points));
% f2 non-uniform grid with every vertex present twice (solver events) and
%    a window that is not aligned with the carrier
uu = cp_lcg(5000, 4242);
h = (0.1 + 2.9 * uu) * (Tw / 100);
t2 = cumsum([0; h]);
t2 = t2(t2 <= 12 * Tw);
t2 = sort([t2; tv; tv]);
y2 = tri(t2);
[pp, av, inf2] = ripple_from_waveform(t2, y2, fs, [0.37 * Tw, 10.9 * Tw]);
ok = abs(pp - Bpp) < 1e-12 && abs(av - A0) < 1e-12 && inf2.n_periods == 10 && ...
    abs(inf2.t_end - inf2.t_start - 10 * Tw) < 1e-15;
res = cp_add(res, 'f_nonuniform_with_vertices', ok, sprintf(['steps %.2f..%.2f us, window [0.37 10.9] Tsw ' ...
    '-> %d periods: pp %.15g, avg %.15g, sparsest period %d pts'], min(h) * 1e6, max(h) * 1e6, ...
    inf2.n_periods, pp, av, inf2.min_points_per_period));
% f3 non-uniform grid without the vertices: bounded underestimate
t3 = cumsum([0; h]);
t3 = t3(t3 <= 12 * Tw);
[pp, av] = ripple_from_waveform(t3, tri(t3), fs, [Tw, 11 * Tw]);
slope = Bpp / ((1 - Dr) * Tw);
bound = 2 * slope * max(h);
ok = pp <= Bpp + 1e-12 && Bpp - pp < bound && abs(av - A0) < 1e-3 * Bpp;
res = cp_add(res, 'f_nonuniform_no_vertices', ok, sprintf(['pp %.5f (1.2 - %.2e, bound %.2e), avg %.6f ' ...
    '(err %.1e)'], pp, Bpp - pp, bound, av, av - A0));
% f4 multi-column, aliasing warning, too-short window, tiny time reversal
[pp, av] = ripple_from_waveform(t1, [tri(t1), 2 * tri(t1) + 1], fs, [Tw, 11 * Tw]);
okm = max(abs(pp - [Bpp, 2 * Bpp])) < 1e-12 && max(abs(av - [A0, 2 * A0 + 1])) < 1e-12;
% (lastwarn is not updated for disabled warnings in Octave: let it print)
fprintf('INFO f: the next warning (aliasing, 10 samples per period) is expected\n');
lastwarn('');
t4 = (0:12 * 10)' * (Tw / 10);
[~, ~, inf4] = ripple_from_waveform(t4, tri(t4), fs);
[~, wid] = lastwarn();
lastwarn('');
ripple_from_waveform(t1, tri(t1), fs);
[~, wid2] = lastwarn();
okw = inf4.aliasing && strcmp(wid, 'ripple_from_waveform:aliasing') && isempty(wid2) && ...
    inf4.min_points_per_period < 20;
try
    ripple_from_waveform(t1, tri(t1), fs, [0, 0.9 * Tw]);
    oke = false;
catch err
    oke = strcmp(err.identifier, 'ripple_from_waveform:window');
end
t5 = t1;
t5(500) = t5(501) + 1e-20;          % rounding-level reversal
[pp5, av5] = ripple_from_waveform(t5, tri(t1), fs, [Tw, 11 * Tw]);
okr = isfinite(pp5) && isfinite(av5);
res = cp_add(res, 'f_columns_warning_errors', okm && okw && oke && okr, sprintf(['multi-column %d, ' ...
    'aliasing warning at 10 pts/period %d (none at 200), short window error %d, tiny reversal ok %d'], ...
    okm, okw, oke, okr));

% ---------------------------------------------------------------- g ------
for m = 1:2
    if m == 1
        Q = PP; tag = 'center';
    else
        Q = PP; Q.pwm = 'trailing'; tag = 'trailing';
    end
    iL = 4.5; vC = 47; tt = []; yy = []; vv = []; imx = -Inf; imn = Inf; vmx = -Inf; vmn = Inf;
    sa = 0; sv = 0; np = 10;
    for k = 1:np
        dk = 0.5 + 0.05 * sin(k);
        [iL, vC, info, w] = plant_switched_step(iL, vC, dk, 20, Vin, Q, 60);
        tt = [tt; (k - 1) * Ts + w.t]; %#ok<AGROW>
        yy = [yy; w.iL]; %#ok<AGROW>
        vv = [vv; w.Vout]; %#ok<AGROW>
        imx = max(imx, info.iL_max); imn = min(imn, info.iL_min);
        vmx = max(vmx, info.Vout_max); vmn = min(vmn, info.Vout_min);
        sa = sa + info.iL_avg; sv = sv + info.Vout_avg;
    end
    [pp, av, ig] = ripple_from_waveform(tt, [yy, vv], P.fs, [0, np * Ts]);
    eg = max(abs([pp - [imx - imn, vmx - vmn], av - [sa, sv] / np]));
    res = cp_add(res, ['g_wave_output_' tag], eg < 1e-9 && ig.n_periods == np, sprintf(['%d periods, ' ...
        'iL pp %.4f A avg %.4f A, Vout pp %.4f V: max |diff| vs info %.1e'], ig.n_periods, pp(1), av(1), ...
        pp(2), eg));
end

% ---------------------------------------------------------------- h ------
try
    so = struct('plant', 'switched', 'nsub', 50);
    ot = sim_boost('cascade_pi', [], P, so);
    so.pwm = 'center';
    oc = sim_boost('cascade_pi', [], P, so);
    oa = sim_boost('cascade_pi', [], P);
    w = (oc.t >= 0.040) & (oc.t < 0.050);
    bc = mean(oc.iL_avg(w) - oc.iL(w));
    bt = mean(ot.iL_avg(w) - ot.iL(w));
    rI = mean(oc.iL_max(w) - oc.iL_min(w));
    res = cp_add(res, 'h_sampling_bias', abs(bc) < 0.05 && abs(bt / 0.6 - 1) < 0.1 && ...
        abs(rI / 1.2 - 1) < 0.1 && ~any(oc.dcm(w)), sprintf(['cascade_pi 40-50 ms: mean(iL_avg - iL sampled) ' ...
        'centre %.4f A (tol 0.05), trailing %.4f A (~dI/2 = 0.6); iL pk-pk %.3f A'], bc, bt, rI));
    offc = abs(mean(oc.Vo_avg(w)) - P.Vref);
    offt = abs(mean(ot.Vo_avg(w)) - P.Vref);
    trk = abs(mean(oc.Vo(w)) - mean(oa.Vo(w)));
    trt = abs(mean(ot.Vo(w)) - mean(oa.Vo(w)));
    res = cp_add(res, 'h_Vo_offset_40_50ms', offc < offt && trk < 0.05 && trk < trt, sprintf(['|mean Vo_avg - ' ...
        'Vref| centre %.3f V < trailing %.3f V; |mean Vo - averaged-plant Vo| centre %.4f V, trailing %.3f V'], ...
        offc, offt, trk, trt));
    sl = struct('plant', 'switched', 'nsub', 50, 'T_end', 0.3, 'load', struct('kind', 'const'));
    ot = sim_boost('cascade_pi', [], P, sl);
    sl.pwm = 'centre';
    oc = sim_boost('cascade_pi', [], P, sl);
    w = (oc.t >= 0.25) & (oc.t < 0.2995);
    ec = mean(oc.Vo_avg(w)) - P.Vref;
    et = mean(ot.Vo_avg(w)) - P.Vref;
    res = cp_add(res, 'h_Vo_offset_steady', abs(ec) < 0.01 && abs(ec) < abs(et) && ...
        strcmp(oc.info.pwm, 'center'), sprintf(['R = 20, 250-300 ms: mean Vo_avg - Vref centre %+.4f V, ' ...
        'trailing %+.4f V (regulates the end-of-OFF sample = ripple top); IL_ref centre %.3f A, trailing %.3f A'], ...
        ec, et, mean(oc.IL_ref(w)), mean(ot.IL_ref(w))));
catch err
    res = cp_add(res, 'h_sim_boost', false, ['sim_boost failed: ' err.message]);
end

% ---------------------------------------------------------------- i ------
try
    g = P.gains_baseline.cascade_pi;
    so = struct('plant', 'switched', 'nsub', 20, 'T_end', 0.02);
    o0 = sim_boost('cascade_pi', g, P, so);
    so.pwm = 'trailing';
    o1 = sim_boost('cascade_pi', g, P, so);
    Pq = P; Pq.pwm = 'center';
    so = rmfield(so, 'pwm');
    o2 = sim_boost('cascade_pi', g, P, setfield(so, 'Pplant', Pq)); %#ok<SFLD>
    so.pwm = 'center';
    o3 = sim_boost('cascade_pi', g, P, so);
    ok_sw = isequaln(rmfield(o0, 'info'), rmfield(o1, 'info')) && isequal(o2.iL, o3.iL) && ...
        ~isequal(o3.iL, o0.iL) && strcmp(o0.info.pwm, 'trailing');
    oa0 = sim_boost('cascade_pi', g, P);
    fprintf('INFO i: the next warning (pwm with the averaged plant) is expected\n');
    lastwarn('');
    oa1 = sim_boost('cascade_pi', g, P, struct('pwm', 'center'));
    [~, wid] = lastwarn();
    ok_avg = isequal(oa0.iL, oa1.iL) && isequal(oa0.Vo, oa1.Vo) && strcmp(wid, 'sim_boost:pwm');
    res = cp_add(res, 'i_sim_boost_defaults', ok_sw && ok_avg, sprintf(['switched default == explicit ' ...
        '''trailing'' (bit-exact), Pplant.pwm honoured: %d; averaged plant ignores pwm (bit-exact, warns): %d'], ...
        ok_sw, ok_avg));
catch err
    res = cp_add(res, 'i_sim_boost_defaults', false, err.message);
end
if ~skip_core
    try
        test_core_equivalence();
        res = cp_add(res, 'i_test_core_equivalence', true, 'all legacy checks PASS');
    catch err
        res = cp_add(res, 'i_test_core_equivalence', false, err.message);
    end
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
    error('test_centre_pwm:failed', '%d check(s) failed', nf);
end
end

% ======================================================================
function res = cp_add(res, name, passed, detail)
res(end + 1) = struct('name', name, 'passed', logical(passed), 'detail', detail);
end

function [info, imin, x1, x0] = cp_settle(P, iL, vC, d, R, Vin, nsub, n)
% n periods from (iL, vC); info of the last one, x0 = state at its start,
% x1 = final state, imin = lowest iL seen
imin = Inf;
for k = 1:n
    x0 = [iL, vC];
    [iL, vC, info] = plant_switched_step(iL, vC, d, R, Vin, P, nsub);
    imin = min([imin, info.iL_min, iL]);
end
x1 = [iL, vC];
end

function A = cp_runall(P, iL, vC, d, R, Vin, nsub, n)
% per period: iL1 vC1 Vout_end iL_avg Vout_avg iL_min iL_max Vout_min Vout_max dcm
A = zeros(n, 10);
for k = 1:n
    [iL, vC, info] = plant_switched_step(iL, vC, d, R, Vin, P, nsub);
    A(k, :) = [iL, vC, info.Vout_end, info.iL_avg, info.Vout_avg, info.iL_min, info.iL_max, ...
        info.Vout_min, info.Vout_max, info.dcm];
end
end

function e = cp_cmp_kernel(P, iL, vC, d, R, Vin, nsub, center)
% max relative difference between the vectorised and the scalar kernel
rL = 0; rC = 0; Ron = 0; VF = 0;
if isfield(P, 'rL'), rL = P.rL; rC = P.rC; Ron = P.Ron; VF = P.VF; end
P.pwm = center;
[a1, a2, info] = plant_switched_step(iL, vC, d, R, Vin, P, nsub);
[b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13] = sw_boost_period(iL, vC, d, R, Vin, ...
    P.L, P.C, P.Ts, rL, rC, Ron, VF, nsub, double(P.sync), center);
x = [a1, a2, info.Vout_end, info.iL_min, info.iL_max, info.Vout_min, info.Vout_max, ...
    info.iL_avg, info.Vout_avg, double(info.dcm), info.t_idle * 1e6, info.iL_mid_on, info.Vout_mid_on];
y = [b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11 * 1e6, b12, b13];
e = max(abs(x - y) ./ max(1, abs(x)));
end

function y = cp_tri(t, T, D, A, B, ph)
% triangle of period T, mean A, pk-pk B: rises for D*T, falls for (1-D)*T;
% valley at t = ph + k*T
x = mod(t - ph, T) / T;
y = zeros(size(t));
r = x < D;
y(r) = A - B / 2 + B * x(r) / D;
y(~r) = A + B / 2 - B * (x(~r) - D) / (1 - D);
end

function tv = cp_tri_vertices(n, T, D, ph)
k = (-1:n)';
tv = [ph + k * T; ph + (k + D) * T];
tv = tv(tv >= 0 & tv <= n * T);
end

function u = cp_lcg(n, seed)
u = zeros(n, 1);
x = seed;
for j = 1:n
    x = mod(16807 * x, 2147483647);
    u(j) = x / 2147483647;
end
end
