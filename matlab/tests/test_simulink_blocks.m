function test_simulink_blocks()
%TEST_SIMULINK_BLOCKS  The MATLAB Function block code reproduces sim_boost exactly.
%   test_simulink_blocks()  prints PASS/FAIL lines and errors at the end if
%   any check failed (about 1-2 min in Octave 8.4).
%   1  blk_profile_eval / blk_scenario == load_profile / vin_profile
%      (all kinds, full time grid, bit-exact)
%   2  make_ctrl_params round trip (emu_decode_cp), defaults, errors;
%      make_profile_vec / make_plant_params contents and errors
%   3  emulate_simulink (model boost_avg executed block by block) ==
%      sim_boost, isequaln on every output field: 5 controllers x baseline
%      and Python-MOPSO gains, ff measured / none, delay 1..3, ADC + noise
%      (seeds 1, 2), load ramp / double_step / const, Vin step / ramp,
%      Pplant L*1.2 C*0.8, outer-loop overrides, initial state, combined;
%      RNG restore, delay FIFO, emu_model_vars, blk_noise_lookup
%   4  emulate_simulink legacy traces == tests/ref/python_traces.csv
%      (bit-exact; SMC within 1e-12 because numpy's tanh differs by 1 ulp)
%   5  model boost_cycle (blk_plant_switched -> sw_boost_period) vs
%      sim_boost 'switched' (plant_switched_step), trailing and centre PWM,
%      cascade_pi and fcsmpc: equal to rounding, explained by a per-period
%      replay of the emulated run through plant_switched_step
%   6  code-generation hygiene of the blk_* files and fixed output sizes
here = fileparts(mfilename('fullpath'));
refdir = fullfile(here, 'ref');
P = boost_params();
JS = core_read_json_exact(fullfile(refdir, 'python_reference.json'));
[h, M] = core_read_csv(fullfile(refdir, 'python_traces.csv'));
ctrls = {'fcsmpc', 'cascade_pi', 'cascade_pid', 'smc', 'single_pi'};
nf = 0;

%% 1. profiles -----------------------------------------------------------
Ts = P.Ts;
N3 = fix(0.3 / Ts);
t = (0:N3 - 1)' * Ts;
lspecs = { ...
    'legacy []', []; ...
    'step (kind only)', struct('kind', 'step'); ...
    'step -25%', struct('kind', 'step', 'pct', 0.25, 't_step', 0.050); ...
    'step -75%', struct('kind', 'step', 'pct', 0.75, 't_step', 0.050); ...
    'step R_new', struct('kind', 'step', 'R0', 16, 'R_new', 9, 't_step', 0.0123); ...
    'step never', struct('kind', 'step', 'pct', 0.0, 't_step', 999); ...
    'const', struct('kind', 'const'); ...
    'const R', struct('kind', 'const', 'R', 13.5); ...
    'ramp default', struct('kind', 'ramp', 't0', 0.050, 't1', 0.060, 'R_end', 10.0); ...
    'ramp custom', struct('kind', 'ramp', 'R0', 18, 't0', 0.0213, 't1', 0.0771, 'R_end', 9.3); ...
    'ramp t0=t1 (NaN at t0)', struct('kind', 'ramp', 't0', 0.05, 't1', 0.05, 'R_end', 12); ...
    'double_step default', struct('kind', 'double_step'); ...
    'double_step custom', struct('kind', 'double_step', 'R0', 25, 't1', 0.01, 'R_mid', 11, 't2', 0.2, 'R_end', 30); ...
    'double_step t2<t1', struct('kind', 'double_step', 't1', 0.08, 't2', 0.04)};
vspecs = { ...
    'const []', []; ...
    'const Vin', struct('kind', 'const', 'Vin', 30); ...
    'step default', struct('kind', 'step'); ...
    'step +20%', struct('kind', 'step', 'pct', 0.20, 't_step', 0.060); ...
    'step -20% V_new', struct('kind', 'step', 'V_new', 19.2, 't_step', 0.03); ...
    'ramp default', struct('kind', 'ramp'); ...
    'ramp custom', struct('kind', 'ramp', 'Vin', 20, 't0', 0.011, 't1', 0.0937, 'V_end', 31)};
for i = 1:size(lspecs, 1)
    sp = lspecs{i, 2};
    PV = make_profile_vec(sp, 'load', P);
    if isempty(sp)
        sp = struct('kind', 'step', 'R0', P.R0, 'R_new', P.R1, 't_step', P.t_step);
    end
    if ~isfield(sp, 'R0')
        sp.R0 = P.R0;
    end
    ref = load_profile(t, sp);
    [v1, v2] = eval_grid(PV, t, Ts);
    nf = nf + report(sprintf('load profile %-20s blk_profile_eval & blk_scenario == load_profile (%d samples)', ...
        lspecs{i, 1}, N3), isequaln(v1, ref) && isequaln(v2, ref), sprintf('max diff %.3g', max(abs(v1 - ref))));
end
for i = 1:size(vspecs, 1)
    sp = vspecs{i, 2};
    PV = make_profile_vec(sp, 'vin', P);
    if isempty(sp)
        sp = struct('kind', 'const');
    end
    if ~isfield(sp, 'Vin')
        sp.Vin = P.Vin;
    end
    ref = vin_profile(t, sp);
    [v1, v2] = eval_grid(PV, t, Ts);
    nf = nf + report(sprintf('vin  profile %-20s blk_profile_eval & blk_scenario == vin_profile  (%d samples)', ...
        vspecs{i, 1}, N3), isequaln(v1, ref) && isequaln(v2, ref), sprintf('max diff %.3g', max(abs(v1 - ref))));
end
nf = nf + report('blk_profile_eval unknown kind -> NaN', isnan(blk_profile_eval(0.01, [7 0 0 0 0 0 0])), '');
[R, Vin, kn] = blk_scenario(1000, make_profile_vec([], 'load', P), make_profile_vec([], 'vin', P), Ts);
nf = nf + report('blk_scenario k = 1000 (t = 0.05): R = R1, Vin = 24, k_next = 1001', ...
    R == P.R1 && Vin == P.Vin && kn == 1001, '');

%% 2. parameter packers --------------------------------------------------
gold = struct('fcsmpc', 1.25, 'cascade_pi', [0.04 15], 'cascade_pid', [0.03 18 5e-6 2e-4], ...
    'smc', [0.3 0.7], 'single_pi', [0.06 4]);
ok = true;
det = '';
for c = 1:5
    cn = ctrls{c};
    o = struct('ff_mode', 'measured', 'delay', mod(c, 4), 'adc_bits', 11, 'KPV', 0.2 + c, ...
        'KIV', 9 + c, 'IL_MAX', 10 + c, 'Vo_adc_range', [1 61], 'iL_adc_range', [-2 20], ...
        'io_adc_range', [0 16], 'vin_adc_range', [2 50]);
    [CP, X0, nm] = make_ctrl_params(cn, gold.(cn), P, o);
    s = emu_decode_cp(CP);
    dI = P.D_ss * (c ~= 1);
    okc = isequal(size(CP), [1 32]) && isa(CP, 'double') && isequal(size(X0), [8 1]) && ...
        strcmp(nm, cn) && strcmp(s.ctrl, cn) && s.cid == c && isequal(s.gains, gold.(cn)) && ...
        all(CP(2 + numel(gold.(cn)):5) == 0) && strcmp(s.ff_mode, 'measured') && s.delay == mod(c, 4) && ...
        s.adc_bits == 11 && s.KPV == 0.2 + c && s.KIV == 9 + c && s.IL_MAX == 10 + c && ...
        s.Vref == P.Vref && s.D_ss == P.D_ss && s.DMIN == P.DMIN && s.DMAX == P.DMAX && ...
        s.L_model == P.L && s.Ts == P.Ts && isequal(s.Vo_adc_range, [1 61]) && ...
        isequal(s.iL_adc_range, [-2 20]) && isequal(s.io_adc_range, [0 16]) && ...
        isequal(s.vin_adc_range, [2 50]) && s.Vin_nom == P.Vin && all(s.reserved == 0) && ...
        isequal(X0, [0; 0; 0; 0; 0; dI; dI; dI]);
    if ~okc
        ok = false;
        det = [det ' ' cn]; %#ok<AGROW>
    end
end
nf = nf + report('make_ctrl_params -> emu_decode_cp round trip (5 controllers, all fields, X0)', ok, det);
[CP, X0] = make_ctrl_params('cascade_pi', [], P);
s = emu_decode_cp(CP);
nf = nf + report('make_ctrl_params defaults: baseline gains, oracle, delay 0, no ADC, P ranges, X0 = [0 0 0 0 0 D_ss D_ss D_ss]', ...
    isequal(s.gains, P.gains_baseline.cascade_pi) && strcmp(s.ff_mode, 'oracle') && s.delay == 0 && ...
    s.adc_bits == 0 && s.KPV == P.KPV && s.KIV == P.KIV && s.IL_MAX == P.IL_MAX && ...
    isequal(s.Vo_adc_range, P.Vo_adc_range) && isequal(s.iL_adc_range, P.iL_adc_range) && ...
    isequal(s.io_adc_range, P.io_adc_range) && isequal(s.vin_adc_range, P.vin_adc_range) && ...
    isequal(X0, [0; 0; 0; 0; 0; P.D_ss; P.D_ss; P.D_ss]), '');
[CPa, ~, nma] = make_ctrl_params('FCS-MPC', [], P, struct('ff_mode', 'NONE'));
[CPb, ~, nmb] = make_ctrl_params('pid', [], P);
nf = nf + report('make_ctrl_params aliases and case (FCS-MPC, pid, NONE) like sim_boost', ...
    strcmp(nma, 'fcsmpc') && CPa(1) == 1 && CPa(6) == 3 && strcmp(nmb, 'cascade_pid') && CPb(1) == 3, '');
cases_err = { ...
    'unknown ctrl', @() make_ctrl_params('lqr', [], P), 'make_ctrl_params:ctrl'; ...
    'gain count', @() make_ctrl_params('cascade_pid', [1 2], P), 'make_ctrl_params:gains'; ...
    'ff_mode', @() make_ctrl_params('smc', [], P, struct('ff_mode', 'ideal')), 'make_ctrl_params:ff_mode'; ...
    'delay 4', @() make_ctrl_params('smc', [], P, struct('delay', 4)), 'make_ctrl_params:delay'; ...
    'delay -1', @() make_ctrl_params('smc', [], P, struct('delay', -1)), 'make_ctrl_params:delay'; ...
    'profile kind', @() make_profile_vec(struct('kind', 'sine'), 'load', P), 'make_profile_vec:kind'; ...
    'vin double_step', @() make_profile_vec(struct('kind', 'double_step'), 'vin', P), 'make_profile_vec:kind'; ...
    'profile which', @() make_profile_vec([], 'R', P), 'make_profile_vec:which'; ...
    'pwm', @() make_plant_params(P, struct('pwm', 'leading')), 'make_plant_params:pwm'; ...
    'emulate delay 4', @() emulate_simulink('cascade_pi', [], P, struct('delay', 4, 'T_end', 0.001)), 'make_ctrl_params:delay'};
for i = 1:size(cases_err, 1)
    [okE, msg] = expect_error(cases_err{i, 2}, cases_err{i, 3});
    nf = nf + report(sprintf('error %-16s -> %s', cases_err{i, 1}, cases_err{i, 3}), okE, msg);
end
PE = P; PE.L = 1.1e-3; PE.C = 2.2e-4; PE.rL = 0.05; PE.rC = 0.02; PE.Ron = 0.01; PE.VF = 0.7; PE.sync = true;
PP1 = make_plant_params(PE, struct('nsub', 40, 'pwm', 'centre'));
PE.pwm = 'center';
PP2 = make_plant_params(PE, struct());
PP3 = make_plant_params(PE, struct('pwm', 'trailing'));
PP0 = make_plant_params(P, []);
nf = nf + report('make_plant_params [L C Ts rL rC Ron VF nsub sync pwm_center], opts.pwm > Pplant.pwm', ...
    isequal(PP1, [1.1e-3 2.2e-4 P.Ts 0.05 0.02 0.01 0.7 40 1 1]) && PP2(10) == 1 && PP2(8) == 100 && ...
    PP3(10) == 0 && isequal(PP0, [P.L P.C P.Ts 0 0 0 0 100 0 0]), mat2str(PP1));
nf = nf + report('make_profile_vec legacy load = [1 t_step R0 R1 0 0 0], vin [] = [0 Vin 0 ...]', ...
    isequal(make_profile_vec([], 'load', P), [1 P.t_step P.R0 P.R1 0 0 0]) && ...
    isequal(make_profile_vec([], 'vin', P), [0 P.Vin 0 0 0 0 0]), '');

%% 3. emulate_simulink (boost_avg) == sim_boost ----------------------------
gp = JS.gains_mopso_python;
gb = JS.gains_baseline;
Pp = P; Pp.L = 1.2 * P.L; Pp.C = 0.8 * P.C;
nz = @(s, extra) merge(struct('noise_std_v', 0.05, 'noise_std_i', 0.02, 'noise_std_io', 0.01, ...
    'noise_std_vin', 0.05, 'adc_bits', 10, 'seed', s), extra);
runs = cell(0, 4);
for c = 1:5
    runs(end + 1, :) = {['legacy base ' ctrls{c}], ctrls{c}, gb.(ctrls{c}), struct()}; %#ok<AGROW>
    runs(end + 1, :) = {['legacy mopso ' ctrls{c}], ctrls{c}, gp.(ctrls{c}), struct()}; %#ok<AGROW>
end
for c = 1:5
    runs(end + 1, :) = {['ff measured ' ctrls{c}], ctrls{c}, gp.(ctrls{c}), struct('ff_mode', 'measured')}; %#ok<AGROW>
end
for c = 1:4
    runs(end + 1, :) = {['ff none ' ctrls{c}], ctrls{c}, gb.(ctrls{c}), struct('ff_mode', 'none')}; %#ok<AGROW>
end
runs = [runs; { ...
    'delay 1 cascade_pi', 'cascade_pi', gp.cascade_pi, struct('delay', 1); ...
    'delay 1 fcsmpc', 'fcsmpc', gp.fcsmpc, struct('delay', 1); ...
    'delay 1 smc', 'smc', gp.smc, struct('delay', 1); ...
    'delay 2 cascade_pid', 'cascade_pid', gp.cascade_pid, struct('delay', 2); ...
    'delay 3 cascade_pi', 'cascade_pi', gp.cascade_pi, struct('delay', 3); ...
    'delay 3 fcsmpc', 'fcsmpc', gp.fcsmpc, struct('delay', 3); ...
    'delay 3 single_pi', 'single_pi', gb.single_pi, struct('delay', 3); ...
    'adc10+noise s1 measured cascade_pi', 'cascade_pi', gp.cascade_pi, nz(1, struct('ff_mode', 'measured')); ...
    'adc10+noise s2 measured cascade_pi', 'cascade_pi', gp.cascade_pi, nz(2, struct('ff_mode', 'measured')); ...
    'adc10+noise s1 measured fcsmpc', 'fcsmpc', gp.fcsmpc, nz(1, struct('ff_mode', 'measured')); ...
    'adc10+noise s2 oracle smc', 'smc', gp.smc, nz(2, struct()); ...
    'adc10+noise s1 none cascade_pid', 'cascade_pid', gb.cascade_pid, nz(1, struct('ff_mode', 'none')); ...
    'adc10+noise s2 single_pi', 'single_pi', gb.single_pi, nz(2, struct()); ...
    'adc10 legacy iL range (cell 12)', 'cascade_pid', gp.cascade_pid, struct('adc_bits', 10, ...
        'noise_std_v', 0.05, 'noise_std_i', 0.02, 'iL_adc_range', P.iL_adc_range_legacy, 'seed', 1); ...
    'adc12 no noise', 'smc', gp.smc, struct('adc_bits', 12, 'ff_mode', 'measured'); ...
    'noise only (Vo, iL) bits 0', 'cascade_pi', gp.cascade_pi, struct('noise_std_v', 0.1, 'noise_std_i', 0.05, 'seed', 1); ...
    'load ramp cascade_pi', 'cascade_pi', gp.cascade_pi, struct('load', struct('kind', 'ramp', 't0', 0.05, 't1', 0.06, 'R_end', 10)); ...
    'load ramp fcsmpc', 'fcsmpc', gp.fcsmpc, struct('load', struct('kind', 'ramp')); ...
    'load double_step cascade_pid', 'cascade_pid', gp.cascade_pid, struct('load', struct('kind', 'double_step')); ...
    'load double_step smc measured', 'smc', gp.smc, struct('load', struct('kind', 'double_step'), 'ff_mode', 'measured'); ...
    'load const', 'cascade_pi', gb.cascade_pi, struct('load', struct('kind', 'const', 'R', 12)); ...
    'load -75% T_end 0.3 (saturated)', 'cascade_pi', gb.cascade_pi, struct('load', struct('kind', 'step', 'pct', 0.75, 't_step', 0.05), 'T_end', 0.3); ...
    'vin step +20% cascade_pi measured', 'cascade_pi', gp.cascade_pi, struct('ff_mode', 'measured', ...
        'load', struct('kind', 'step', 'pct', 0.0, 't_step', 999), 'vin', struct('kind', 'step', 'pct', 0.2, 't_step', 0.06)); ...
    'vin step +20% fcsmpc oracle', 'fcsmpc', gp.fcsmpc, struct('vin', struct('kind', 'step', 'pct', 0.2, 't_step', 0.06)); ...
    'vin ramp smc measured', 'smc', gp.smc, struct('ff_mode', 'measured', 'vin', struct('kind', 'ramp', 'V_end', 20)); ...
    'Pplant L*1.2 C*0.8 cascade_pid', 'cascade_pid', gp.cascade_pid, struct('Pplant', Pp); ...
    'Pplant L*1.2 C*0.8 fcsmpc', 'fcsmpc', gp.fcsmpc, struct('Pplant', Pp); ...
    'Pplant L*1.2 C*0.8 smc', 'smc', gp.smc, struct('Pplant', Pp); ...
    'outer KPV/KIV/IL_MAX override', 'cascade_pi', gp.cascade_pi, struct('KPV', 0.3, 'KIV', 30, 'IL_MAX', 10); ...
    'iL0/Vo0, T_end 0.03', 'fcsmpc', gp.fcsmpc, struct('iL0', 3, 'Vo0', 40, 'T_end', 0.03); ...
    'combined pid', 'cascade_pid', gp.cascade_pid, nz(7, struct('ff_mode', 'measured', 'delay', 2, 'adc_bits', 12, ...
        'vin', struct('kind', 'ramp', 'V_end', 28), 'load', struct('kind', 'double_step'), 'Pplant', Pp, 'KPV', 0.2)); ...
    'combined fcsmpc', 'fcsmpc', gp.fcsmpc, nz(8, struct('ff_mode', 'measured', 'delay', 1, ...
        'vin', struct('kind', 'step', 'pct', -0.1, 't_step', 0.03), 'load', struct('kind', 'ramp'), 'Pplant', Pp))}];
for i = 1:size(runs, 1)
    a = sim_boost(runs{i, 2}, runs{i, 3}, P, runs{i, 4});
    b = emulate_simulink(runs{i, 2}, runs{i, 3}, P, runs{i, 4});
    [same, msg] = compare_out(a, b);
    nf = nf + report(sprintf('boost_avg == sim_boost  %-36s (N=%d, isequaln all fields)', runs{i, 1}, numel(a.t)), ...
        same, msg);
end
rng(4321, 'twister');
r1 = rand;
rng(4321, 'twister');
emulate_simulink('cascade_pi', [], P, nz(3, struct('T_end', 0.002)));
r2 = rand;
nf = nf + report('emulate_simulink restores the caller RNG state', r1 == r2, '');
V = emu_model_vars('fcsmpc', gp.fcsmpc, P, nz(5, struct('ff_mode', 'measured', 'T_end', 0.02)));
okn = true;
for k = 0:V.N + 1
    nk = blk_noise_lookup(k, V.noise);
    if k < V.N
        okn = okn && isequal(nk, V.noise(:, k + 1));
    else
        okn = okn && isequal(nk, zeros(4, 1));
    end
end
nf = nf + report('blk_noise_lookup(k, NZ) == noise(:, k+1), zeros outside (exact alternative to the Inport)', okn, '');
a = emulate_simulink('fcsmpc', gp.fcsmpc, P, nz(5, struct('ff_mode', 'measured', 'T_end', 0.02)));
nf = nf + report('emu_model_vars == emulate_simulink info (CP X0 LP VP PP XP0 model) and noise = sd .* randn(seed)', ...
    isequal(V.CP, a.info.CP) && isequal(V.X0, a.info.X0) && isequal(V.LP, a.info.LP) && ...
    isequal(V.VP, a.info.VP) && isequal(V.PP, a.info.PP) && isequal(V.XP0, a.info.XP0) && ...
    strcmp(V.model, a.info.model) && V.has_noise && V.N == 400 && isequal(V.t, a.t), '');
b = emulate_simulink('cascade_pi', gp.cascade_pi, P, struct('delay', 3));
nf = nf + report('delay 3: d(1:3) = D_ss, d(k+3) = d_cmd(k) = xc_next(5)', all(b.d(1:3) == P.D_ss) && ...
    isequal(b.d(4:end), b.d_cmd(1:end - 3)), '');

%% 4. Python reference traces --------------------------------------------
sets = {'base', 'opt'};
gsets = {'gains_baseline', 'gains_mopso_python'};
for s = 1:2
    for c = 1:5
        cn = ctrls{c};
        b = emulate_simulink(cn, JS.(gsets{s}).(cn), P);
        pre = [cn '_' sets{s} '_'];
        dI = max(abs(b.iL - col(h, M, [pre 'iL'])));
        dV = max(abs(b.Vo - col(h, M, [pre 'Vo'])));
        dU = 0;
        if c == 1
            u = col(h, M, [pre 'u']);
            dU = max(abs(b.u(1:end - 1) - u(1:end - 1)));
        end
        dmax = max([dI dV dU]);
        if strcmp(cn, 'smc')
            okp = dmax <= 1e-12;
            tag = '  (numpy tanh 1-ulp, tol 1e-12)';
        else
            okp = dmax == 0;
            tag = '  (bit-exact required)';
        end
        nf = nf + report(sprintf('python_traces.csv %-11s %-4s max|diff| iL=%.3g Vo=%.3g u=%.3g%s', ...
            cn, sets{s}, dI, dV, dU, tag), okp, '');
    end
end

%% 5. boost_cycle (switched plant) ----------------------------------------
if nargin('sw_boost_period') < 15
    nf = nf + report('sw_boost_period has the 15th input pwm_center', false, ...
        sprintf('nargin = %d; blk_plant_switched needs 15 inputs', nargin('sw_boost_period')));
else
    simtxt = fileread(which('sim_boost'));
    sim_opts_pwm = ~isempty(strfind(simtxt, '''pwm'''));
    sw = {'cascade_pi', gp.cascade_pi; 'fcsmpc', gp.fcsmpc};
    pw = {'trailing', 'center'};
    for c = 1:size(sw, 1)
        for p = 1:2
            Pq = P;
            Pq.pwm = pw{p};
            o = struct('plant', 'switched', 'Pplant', Pq);
            if p == 2 && c == 2
                o.nsub = 60;
            end
            a = sim_boost(sw{c, 1}, sw{c, 2}, P, o);
            b = emulate_simulink(sw{c, 1}, sw{c, 2}, P, o);
            [same, msg, dmax] = compare_out(a, b);
            fx = {'t', 'R', 'Vin', 'dcm'};
            okx = true;
            for j = 1:numel(fx)
                okx = okx && isequal(a.(fx{j}), b.(fx{j}));
            end
            if c == 2
                okx = okx && isequal(a.d, b.d);     % FCS-MPC gate sequence identical
            end
            nf = nf + report(sprintf(['boost_cycle vs sim_boost switched %-10s %-8s max|diff| %.2g ' ...
                '(t R Vin dcm%s identical)%s'], sw{c, 1}, pw{p}, dmax, ...
                ternary(c == 2, ' gates', ''), ternary(same, '  BIT-EXACT', '')), ...
                okx && dmax <= 1e-8, msg);
            % replay every period of the emulated run through plant_switched_step
            nsub = getfield_def(o, 'nsub', 100);
            Nn = numel(b.t);
            e = zeros(Nn - 1, 1);
            for k = 1:Nn - 1
                [i1, v1, inf1] = plant_switched_step(b.iL(k), b.Vc(k), b.d(k), b.R(k), b.Vin(k), Pq, nsub);
                ref = [i1, v1, inf1.Vout_end, inf1.Vout_min, inf1.Vout_max, inf1.iL_min, inf1.iL_max, ...
                    inf1.iL_avg, inf1.Vout_avg];
                got = [b.iL(k + 1), b.Vc(k + 1), b.Vo(k + 1), b.Vo_min(k), b.Vo_max(k), b.iL_min(k), ...
                    b.iL_max(k), b.iL_avg(k), b.Vo_avg(k)];
                e(k) = max(abs(got - ref) ./ max(1, abs(ref)));
                if inf1.dcm ~= b.dcm(k)
                    e(k) = Inf;
                end
            end
            nf = nf + report(sprintf(['  per-period replay through plant_switched_step: max rel diff %.2g ' ...
                '(kernel rounding only)'], max(e)), max(e) <= 1e-11, '');
        end
    end
    if sim_opts_pwm
        o = struct('plant', 'switched', 'pwm', 'center', 'nsub', 50, 'T_end', 0.02);
        a = sim_boost('cascade_pi', gp.cascade_pi, P, o);
        b = emulate_simulink('cascade_pi', gp.cascade_pi, P, o);
        Pq = P; Pq.pwm = 'center';
        c2 = emulate_simulink('cascade_pi', gp.cascade_pi, P, struct('plant', 'switched', 'Pplant', Pq, ...
            'nsub', 50, 'T_end', 0.02));
        [~, ~, dmax] = compare_out(a, b);
        nf = nf + report(sprintf('opts.pwm = ''center'' (sim_boost option): max|diff| %.2g, same as Pplant.pwm', dmax), ...
            dmax <= 1e-8 && isequal(b.Vo, c2.Vo), '');
    else
        fprintf('INFO  sim_boost has no opts.pwm: centre PWM selected through Pplant.pwm (read by plant_switched_step)\n');
    end
end

%% 6. code-generation hygiene ---------------------------------------------
blks = {'blk_controller', 'blk_scenario', 'blk_profile_eval', 'blk_plant_avg', 'blk_plant_switched', ...
    'blk_noise_lookup'};
deps = {'sense_adc', 'round_half_even', 'ctrl_outer_pi', 'ctrl_inner_pi', 'ctrl_inner_pid', 'ctrl_smc', ...
    'ctrl_single_pi', 'ctrl_fcsmpc', 'plant_avg_step', 'sw_boost_period'};
% whole-word patterns (regexp) that are not allowed in block code
% (explicit [^A-Za-z0-9_] boundaries: MATLAB has no \b, PCRE has no \< \>)
B = '(^|[^A-Za-z0-9_])';
E = '([^A-Za-z0-9_]|$)';
forbid = {[B 'struct\s*\('], '\{', [B 'try' E], [B 'catch' E], [B 'fprintf' E], [B 'disp' E], ...
    [B 'varargin' E], [B 'varargout' E], [B 'nargin' E], [B 'end\s*\+\s*1'], [B 'eval\s*\('], ...
    [B 'global' E], '@', [B 'cell\s*\('], [B 'containers\.'], [B 'persistent' E]};
bad = {};
for i = 1:numel(blks)
    txt = fileread(which(blks{i}));
    if isempty(strfind(txt, '%#codegen'))
        bad{end + 1} = [blks{i} ':no %#codegen']; %#ok<AGROW>
    end
    code = strip_comments(txt);
    for j = 1:numel(forbid)
        if ~isempty(regexp(code, forbid{j}, 'once'))
            bad{end + 1} = [blks{i} ':' forbid{j}]; %#ok<AGROW>
        end
    end
end
for i = 1:numel(deps)
    if isempty(strfind(fileread(which(deps{i})), '%#codegen'))
        bad{end + 1} = [deps{i} ':no %#codegen']; %#ok<AGROW>
    end
end
nf = nf + report('blk_* files: %#codegen, no struct/cell/try/fprintf/varargin/nargin/growth; callees %#codegen', ...
    isempty(bad), strjoin(bad, ' '));
[CP, X0] = make_ctrl_params('cascade_pid', [], P, struct('ff_mode', 'measured', 'delay', 2));
[d, ILr, xn, me] = blk_controller([47; 4.5; 2.35; 24; 20], [0; 0; 0; 0], X0, CP);
xa = blk_plant_avg([4.5; 47], d, 20, 24, make_plant_params(P, []));
[xs, en] = blk_plant_switched([4.5; 47; 47], d, 20, 24, make_plant_params(P, struct('nsub', 20)));
[R, Vin, kn] = blk_scenario(3, make_profile_vec([], 'load', P), make_profile_vec([], 'vin', P), Ts);
v = blk_profile_eval(0.01, make_profile_vec(struct('kind', 'ramp'), 'load', P));
sz = {d, [1 1]; ILr, [1 1]; xn, [8 1]; me, [4 1]; xa, [2 1]; xs, [3 1]; en, [7 1]; R, [1 1]; ...
    Vin, [1 1]; kn, [1 1]; v, [1 1]};
oks = true;
for i = 1:size(sz, 1)
    oks = oks && isequal(size(sz{i, 1}), sz{i, 2}) && isa(sz{i, 1}, 'double') && isreal(sz{i, 1});
end
[~, ILs] = blk_controller([47; 4.5; 2.35; 24; 20], [0; 0; 0; 0], [0; 0; 0; 0; 0; 0.5; 0.5; 0.5], ...
    make_ctrl_params('single_pi', [], P));
nf = nf + report('block outputs fixed size/double: d 1x1, IL_ref 1x1, xc_next 8x1, meas 4x1, xp 2x1/3x1, env 7x1', ...
    oks && isnan(ILs), '');

if nf > 0
    error('test_simulink_blocks:failed', '%d check(s) FAILED', nf);
end
fprintf('test_simulink_blocks: all checks PASSED\n');
end

% ------------------------------------------------------------------------
function [v1, v2] = eval_grid(PV, t, Ts)
% blk_profile_eval on t and blk_scenario on k = 0..N-1 (load slot)
n = numel(t);
v1 = zeros(n, 1);
v2 = zeros(n, 1);
VP = [0 1 0 0 0 0 0];
for k = 1:n
    v1(k) = blk_profile_eval(t(k), PV);
    [v2(k), ~, ~] = blk_scenario(k - 1, PV, VP, Ts);
end
end

function [same, msg, dmax] = compare_out(a, b)
% every sim_boost field (except info) must exist in b and be isequaln
f = fieldnames(a);
bad = {};
dmax = 0;
for i = 1:numel(f)
    fn = f{i};
    if strcmp(fn, 'info')
        continue;
    end
    if ~isfield(b, fn)
        bad{end + 1} = [fn ':missing']; %#ok<AGROW>
        dmax = Inf;
        continue;
    end
    x = a.(fn);
    y = b.(fn);
    if ~isequaln(x, y)
        if isequal(size(x), size(y))
            dx = double(x(:)) - double(y(:));
            nn = isnan(x(:)) & isnan(y(:));
            dd = max([0; abs(dx(~nn))]);
            if any(isnan(x(:)) ~= isnan(y(:)))
                dd = Inf;
            end
        else
            dd = Inf;
        end
        dmax = max(dmax, dd);
        bad{end + 1} = sprintf('%s:%.3g', fn, dd); %#ok<AGROW>
    end
end
inf_f = {'ctrl', 'gains', 'ff_mode', 'delay', 'plant', 'nsub', 'KPV', 'KIV', 'IL_MAX', 'noise_std', ...
    'adc_bits', 'seed', 'N', 'Ts', 'load', 'vin'};
if isfield(a.info, 'pwm')
    inf_f{end + 1} = 'pwm';
end
for i = 1:numel(inf_f)
    if ~isequal(a.info.(inf_f{i}), b.info.(inf_f{i}))
        bad{end + 1} = ['info.' inf_f{i}]; %#ok<AGROW>
    end
end
same = isempty(bad);
msg = strjoin(bad, ' ');
end

function s = merge(s, extra)
f = fieldnames(extra);
for i = 1:numel(f)
    s.(f{i}) = extra.(f{i});
end
end

function v = getfield_def(s, name, default)
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

function [ok, msg] = expect_error(fh, id)
ok = false;
msg = 'no error raised';
try
    fh();
catch err
    ok = strcmp(err.identifier, id);
    msg = ['got ' err.identifier ': ' err.message];
end
end

function code = strip_comments(txt)
% drop comment lines and trailing % comments (blk files have no % in strings)
lines = regexp(txt, '\r?\n', 'split');
for i = 1:numel(lines)
    p = strfind(lines{i}, '%');
    if ~isempty(p)
        lines{i} = lines{i}(1:p(1) - 1);
    end
end
code = strjoin(lines, sprintf('\n'));
end

function v = col(h, M, name)
j = find(strcmp(h, name));
if numel(j) ~= 1
    error('test_simulink_blocks:col', 'column %s not found', name);
end
v = M(:, j);
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
