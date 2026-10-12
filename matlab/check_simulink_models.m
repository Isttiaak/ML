function R = check_simulink_models(copts)
%CHECK_SIMULINK_MODELS  Validate the Simulink models boost_avg / boost_cycle.
%   R = check_simulink_models()        R = check_simulink_models(copts)
%   Builds the models if they are missing, runs them through
%   sim_boost_simulink and prints a PASS/FAIL table:
%     (a)  boost_avg vs sim_boost, legacy scenario, 5 controllers x
%          baseline / Python-MOPSO gains: max|diff| of Vo iL d IL_ref R Vin
%          Vo_meas iL_meas below 1e-9 (bit-exact expected; SMC within 1e-12
%          relative because of tanh)
%     (a2) non-legacy scenarios in ONE Fast Restart batch (load double step /
%          ramp / -75 %, Vin step / ramp, Pplant L*1.2 C*0.8, ff measured /
%          none, delay 1-3, outer-loop overrides, initial state, different
%          T_end): proves that LP VP PP XP0 CP X0 and StopTime are tunable
%     (b)  boost_avg vs tests/ref/python_traces.csv (notebook traces)
%     (c)  the same SimulationInput run twice under Fast Restart (inside one
%          batch and in a second call) gives identical results: all states
%          are reset between runs
%     (d)  ff measured + delay 1 + 10-bit ADC + sensor noise (seeds 1, 2) vs
%          sim_boost with the same options: the root-Inport noise equals
%          sim_boost's noise stream
%     (e)  boost_cycle vs sim_boost(plant 'switched'), centre and trailing PWM
%          and with parasitics: max|diff| below 1e-8 (sw_boost_period vs
%          plant_switched_step differ by rounding only), dcm flags identical
%   copts (optional struct):
%     folder      folder of / for boost_avg.slx and boost_cycle.slx (default:
%                 where they are on the path, else pwd)
%     rebuild     false (default) | true: rebuild the models first
%     cycle       true (default): include (e)
%     use_parsim  false (default)
%     backend     'simulink' (default) | 'emulate' (dry run of this script
%                 without Simulink: the model is replaced by emulate_simulink;
%                 runs in Octave)
%   R: struct array with fields name, pass, detail (one per table row).
%   UNTESTED here (no MATLAB) with backend 'simulink'; the 'emulate' dry run
%   passes in Octave 8.4.

if nargin < 1 || isempty(copts)
    copts = struct();
end
backend = lower(getopt(copts, 'backend', 'simulink'));
do_cycle = getopt(copts, 'cycle', true);
P = boost_params();
here = fileparts(mfilename('fullpath'));
tdir = fullfile(here, 'tests');
if exist(tdir, 'dir') == 7 && isempty(which('core_read_csv'))
    addpath(tdir);
end
rows = cell(0, 3);
ctrls = {'fcsmpc', 'cascade_pi', 'cascade_pid', 'smc', 'single_pi'};
fprintf('check_simulink_models (backend %s)\n', backend);

% ---- reference data (gains and Python traces)
JS = [];
try
    JS = core_read_json_exact(fullfile(tdir, 'ref', 'python_reference.json'));
catch err
    fprintf('  note: python_reference.json not read (%s): MOPSO gains and (b) skipped\n', err.message);
end
H = {};
M = [];
try
    [H, M] = core_read_csv(fullfile(tdir, 'ref', 'python_traces.csv'));
catch err
    fprintf('  note: python_traces.csv not read (%s): (b) skipped\n', err.message);
end
gsets = {'base', 'gains_baseline'};
if ~isempty(JS)
    gsets = [gsets; {'opt', 'gains_mopso_python'}];
    gp = JS.gains_mopso_python;
    gb = JS.gains_baseline;
else
    gp = P.gains_baseline;
    gb = P.gains_baseline;
end

% ---- models
simcfg = struct('backend', backend, 'use_parsim', getopt(copts, 'use_parsim', false), ...
    'fast_restart', true);
avg_ok = true;
cyc_ok = do_cycle;
mfold = struct('boost_avg', '', 'boost_cycle', '');
if strcmp(backend, 'simulink')
    if isempty(which('new_system'))
        error('check_simulink_models:simulink', ...
            'Simulink not found; use check_simulink_models(struct(''backend'', ''emulate'')) for a dry run');
    end
    folder = getopt(copts, 'folder', '');
    rebuild = getopt(copts, 'rebuild', false);
    names = {'boost_avg'};
    if do_cycle
        names{end + 1} = 'boost_cycle';
    end
    for i = 1:numel(names)
        [fld, found] = locate_model(names{i}, folder);
        if rebuild || ~found
            try
                build_boost_models(strrep(names{i}, 'boost_', ''), fld);
                rows = add(rows, sprintf('build %s.slx in %s', names{i}, fld), true, '');
            catch err
                rows = add(rows, sprintf('build %s.slx', names{i}), false, err.message);
                if i == 1
                    avg_ok = false;
                else
                    cyc_ok = false;
                end
                continue;
            end
        end
        mfold.(names{i}) = fld;
        if ~any(strcmp(strsplit(path, pathsep), fld))
            addpath(fld);
        end
    end
end

% ---- (a) + (b) legacy runs, boost_avg
if avg_ok
    try
        cl = {};
        gl = {};
        tag = {};
        for s = 1:size(gsets, 1)
            for c = 1:numel(ctrls)
                cl{end + 1} = ctrls{c}; %#ok<AGROW>
                gs = JS_or_P(JS, P, gsets{s, 2});
                gl{end + 1} = gs.(ctrls{c}); %#ok<AGROW>
                tag{end + 1} = gsets{s, 1}; %#ok<AGROW>
            end
        end
        fprintf('  (a) %d legacy runs on boost_avg ...\n', numel(cl));
        outs = sim_boost_simulink(cl, gl, P, struct(), mcfg(simcfg, mfold, 'boost_avg'));
        fa = {'Vo', 'iL', 'd', 'IL_ref', 'R', 'Vin', 'Vo_meas', 'iL_meas'};
        for i = 1:numel(cl)
            nm = sprintf('(a) legacy %-11s %-4s boost_avg vs sim_boost', cl{i}, tag{i});
            if ~outs(i).ok
                rows = add(rows, nm, false, outs(i).error);
                continue;
            end
            ref = sim_boost(cl{i}, gl{i}, P);
            [dm, wf] = maxdiff(ref, outs(i), fa);
            rows = add(rows, nm, dm <= 1e-9, diffstr(dm, wf));
            if ~isempty(M)
                pre = [cl{i} '_' tag{i} '_'];
                dI = max(abs(outs(i).iL - col(H, M, [pre 'iL'])));
                dV = max(abs(outs(i).Vo - col(H, M, [pre 'Vo'])));
                dU = 0;
                if strcmp(cl{i}, 'fcsmpc')
                    u = col(H, M, [pre 'u']);
                    dU = max(abs(outs(i).d(1:end - 1) - u(1:end - 1)));
                end
                db = max([dI dV dU]);
                rows = add(rows, sprintf('(b) python_traces.csv %-11s %-4s iL Vo%s', cl{i}, tag{i}, ...
                    ternary(strcmp(cl{i}, 'fcsmpc'), ' u', '')), db <= 1e-9, diffstr(db, ''));
            end
        end
    catch err
        rows = add(rows, '(a)/(b) legacy runs', false, err.message);
    end

    % ---- (a2) non-legacy scenarios, one Fast Restart batch
    try
        Pp = P;
        Pp.L = 1.2 * P.L;
        Pp.C = 0.8 * P.C;
        sc = { ...
            'pid double_step Pplant ff none delay 2', 'cascade_pid', gp.cascade_pid, ...
                struct('load', struct('kind', 'double_step'), 'Pplant', Pp, 'ff_mode', 'none', 'delay', 2); ...
            'fcsmpc Vin ramp ff measured KPV/KIV iL0/Vo0', 'fcsmpc', gp.fcsmpc, ...
                struct('vin', struct('kind', 'ramp', 'V_end', 28), 'ff_mode', 'measured', 'KPV', 0.2, ...
                'KIV', 15, 'IL_MAX', 11, 'iL0', 3, 'Vo0', 40); ...
            'single_pi load ramp delay 3', 'single_pi', gb.single_pi, ...
                struct('load', struct('kind', 'ramp'), 'delay', 3); ...
            'smc Vin step load const ff measured', 'smc', gp.smc, ...
                struct('vin', struct('kind', 'step', 'pct', 0.2, 't_step', 0.06), ...
                'load', struct('kind', 'const', 'R', 12), 'ff_mode', 'measured'); ...
            'pi load -75% T_end 0.15 delay 1', 'cascade_pi', gb.cascade_pi, ...
                struct('load', struct('kind', 'step', 'pct', 0.75, 't_step', 0.05), 'T_end', 0.15, 'delay', 1)};
        fprintf('  (a2) %d scenario runs in one Fast Restart batch ...\n', size(sc, 1));
        outs = sim_boost_simulink(sc(:, 2)', sc(:, 3)', P, sc(:, 4)', mcfg(simcfg, mfold, 'boost_avg'));
        fa = {'Vo', 'iL', 'd', 'IL_ref', 'IL_ff', 'R', 'Vin', 'Vo_meas', 'iL_meas'};
        for i = 1:size(sc, 1)
            nm = sprintf('(a2) %-44s vs sim_boost', sc{i, 1});
            if ~outs(i).ok
                rows = add(rows, nm, false, outs(i).error);
                continue;
            end
            ref = sim_boost(sc{i, 2}, sc{i, 3}, P, sc{i, 4});
            [dm, wf] = maxdiff(ref, outs(i), fa);
            rows = add(rows, nm, dm <= 1e-9 && numel(ref.t) == numel(outs(i).t), ...
                sprintf('%s, N = %d', diffstr(dm, wf), numel(outs(i).t)));
        end
    catch err
        rows = add(rows, '(a2) scenario batch', false, err.message);
    end

    % ---- (c) Fast Restart state reset
    try
        fprintf('  (c) Fast Restart repeatability ...\n');
        A = {'cascade_pid', gp.cascade_pid, struct('ff_mode', 'none', 'delay', 1)};
        B = {'fcsmpc', gp.fcsmpc, struct('iL0', 2, 'Vo0', 45)};
        o1 = sim_boost_simulink({A{1}, B{1}, A{1}}, {A{2}, B{2}, A{2}}, P, {A{3}, B{3}, A{3}}, ...
            mcfg(simcfg, mfold, 'boost_avg'));
        o2 = sim_boost_simulink(A{1}, {A{2}}, P, A{3}, mcfg(simcfg, mfold, 'boost_avg'));
        fc = {'t', 'Vo', 'iL', 'd', 'IL_ref', 'R', 'Vin', 'Vo_meas', 'iL_meas'};
        ok1 = o1(1).ok && o1(3).ok && same_fields(o1(1), o1(3), fc);
        ok2 = o1(1).ok && o2(1).ok && same_fields(o1(1), o2(1), fc);
        rows = add(rows, '(c) run A, B, A in one batch: run 3 identical to run 1', ok1, o1(3).error);
        rows = add(rows, '(c) run A again in a second call: identical', ok2, o2(1).error);
    catch err
        rows = add(rows, '(c) Fast Restart repeatability', false, err.message);
    end

    % ---- (d) measured ff + delay + ADC + noise
    try
        fprintf('  (d) sensor noise / ADC runs ...\n');
        nz = @(seed) struct('ff_mode', 'measured', 'delay', 1, 'adc_bits', 10, 'noise_std_v', 0.05, ...
            'noise_std_i', 0.02, 'noise_std_io', 0.01, 'noise_std_vin', 0.05, 'seed', seed);
        dn = { ...
            'cascade_pi seed 1', 'cascade_pi', gp.cascade_pi, nz(1); ...
            'cascade_pi seed 2', 'cascade_pi', gp.cascade_pi, nz(2); ...
            'fcsmpc     seed 1', 'fcsmpc', gp.fcsmpc, nz(1); ...
            'smc        seed 3', 'smc', gp.smc, nz(3)};
        outs = sim_boost_simulink(dn(:, 2)', dn(:, 3)', P, dn(:, 4)', mcfg(simcfg, mfold, 'boost_avg'));
        fa = {'Vo', 'iL', 'd', 'IL_ref', 'IL_ff', 'R', 'Vin', 'Vo_meas', 'iL_meas'};
        for i = 1:size(dn, 1)
            nm = sprintf('(d) measured ff + delay 1 + ADC 10 bit + noise %-18s', dn{i, 1});
            if ~outs(i).ok
                rows = add(rows, nm, false, outs(i).error);
                continue;
            end
            ref = sim_boost(dn{i, 2}, dn{i, 3}, P, dn{i, 4});
            [dm, wf] = maxdiff(ref, outs(i), fa);
            rows = add(rows, nm, dm <= 1e-9, diffstr(dm, wf));
        end
        if outs(1).ok && outs(2).ok
            rows = add(rows, '(d) seeds 1 and 2 give different measurements (noise is applied)', ...
                ~isequal(outs(1).Vo_meas, outs(2).Vo_meas), '');
        end
    catch err
        rows = add(rows, '(d) noise runs', false, err.message);
    end
end

% ---- (e) boost_cycle
if do_cycle && cyc_ok
    try
        fprintf('  (e) boost_cycle runs ...\n');
        Pq = P;
        Pq.rL = 0.05;
        Pq.rC = 0.02;
        Pq.Ron = 0.01;
        Pq.VF = 0.7;
        ce = { ...
            'cascade_pi centre PWM', 'cascade_pi', gp.cascade_pi, struct('plant', 'switched', 'pwm', 'center'); ...
            'cascade_pi trailing PWM', 'cascade_pi', gp.cascade_pi, struct('plant', 'switched', 'pwm', 'trailing'); ...
            'fcsmpc centre PWM nsub 60', 'fcsmpc', gp.fcsmpc, struct('plant', 'switched', 'pwm', 'center', 'nsub', 60); ...
            'cascade_pi centre rL rC Ron VF', 'cascade_pi', gp.cascade_pi, ...
                struct('plant', 'switched', 'pwm', 'center', 'Pplant', Pq)};
        outs = sim_boost_simulink(ce(:, 2)', ce(:, 3)', P, ce(:, 4)', mcfg(simcfg, mfold, 'boost_cycle'));
        fe = {'Vo', 'iL', 'd', 'R', 'Vin', 'Vc', 'Vo_min', 'Vo_max', 'iL_min', 'iL_max', 'iL_avg', 'Vo_avg'};
        for i = 1:size(ce, 1)
            nm = sprintf('(e) boost_cycle %-30s vs sim_boost switched', ce{i, 1});
            if ~outs(i).ok
                rows = add(rows, nm, false, outs(i).error);
                continue;
            end
            ref = sim_boost(ce{i, 2}, ce{i, 3}, P, ce{i, 4});
            [dm, wf] = maxdiff(ref, outs(i), fe);
            okd = isequal(ref.dcm, outs(i).dcm);
            if strcmp(ce{i, 2}, 'fcsmpc')
                okd = okd && isequal(ref.d, outs(i).d);
            end
            rows = add(rows, nm, dm <= 1e-8 && okd, sprintf('%s, dcm%s %s', diffstr(dm, wf), ...
                ternary(strcmp(ce{i, 2}, 'fcsmpc'), '/gates', ''), ternary(okd, 'identical', 'DIFFER')));
        end
    catch err
        rows = add(rows, '(e) boost_cycle runs', false, err.message);
    end
elseif do_cycle
    rows = add(rows, '(e) boost_cycle', false, 'model not available');
end

% ---- table
np = sum(cell2mat(rows(:, 2)));
nr = size(rows, 1);
fprintf('\n%-5s %-78s %s\n', 'RES', 'CHECK', 'DETAIL');
for i = 1:nr
    fprintf('%-5s %-78s %s\n', ternary(rows{i, 2}, 'PASS', 'FAIL'), rows{i, 1}, rows{i, 3});
end
fprintf('\n%d of %d checks passed (backend %s)\n', np, nr, backend);
names = rows(:, 1);
if np == nr
    fprintf(['Next steps:\n' ...
        '  1. Use sim_boost_simulink for batch runs (Fast Restart; simcfg.use_parsim = true\n' ...
        '     with the Parallel Computing Toolbox), e.g. inside the MOPSO objective.\n' ...
        '  2. Run check_simscape_block_params to see the parameter names of the Simscape\n' ...
        '     blocks in your release, build the Simscape power stage by hand and validate it\n' ...
        '     open loop with simscape_compare_openloop.\n']);
else
    fprintf('What to look at:\n');
    if any_fail(rows, '(a') || any_fail(rows, '(b')
        fprintf(['  (a)/(b): differences ~1e-12..1e-9 are compiler rounding (fused multiply-add, libm);\n' ...
            '    large ones mean wiring or parameters: rebuild with\n' ...
            '    check_simulink_models(struct(''rebuild'', true)) and read the MANUAL STEP lines.\n']);
    end
    if any_fail(rows, '(c')
        fprintf(['  (c): a state or parameter is not reset/tunable: Unit Delay ICs must be X0 / XP0 / 0,\n' ...
            '    CP LP VP PP must be Parameter data (Tunable) of the MATLAB Function blocks.\n']);
    end
    if any_fail(rows, '(d')
        fprintf(['  (d): the noise does not reach the controller as in sim_boost: root Inport ''noise''\n' ...
            '    (width 4, sample time Ts) must be port 2 of Controller; the Simulink time of\n' ...
            '    sample k must be exactly k*Ts.  Exact alternative: replace the Inport by a MATLAB\n' ...
            '    Function n = blk_noise_lookup(k, NZ) fed by the counter k (NZ = 4 x N parameter).\n']);
    end
    if any_fail(rows, '(e')
        fprintf('  (e): check the Plant block of boost_cycle (blk_plant_switched, PP from make_plant_params).\n');
    end
    if any(strncmp(names, 'build', 5) & ~cell2mat(rows(:, 2)))
        fprintf('  build: see the messages of build_boost_models above.\n');
    end
end
R = struct('name', rows(:, 1), 'pass', rows(:, 2), 'detail', rows(:, 3));
end

% ========================================================================
function [fld, found] = locate_model(name, folder)
found = false;
if bdIsLoaded(name)
    % already in memory (e.g. a hand-built model): use it, never rebuild it
    found = true;
    fld = '';
    try
        fld = fileparts(get_param(name, 'FileName'));
    catch
    end
    if isempty(fld)
        fld = pwd;
    end
    return;
end
if ~isempty(folder)
    fld = folder;
    found = exist(fullfile(folder, [name '.slx']), 'file') == 2;
    return;
end
p = which([name '.slx']);
if ~isempty(p)
    fld = fileparts(p);
    found = true;
else
    fld = pwd;
    found = exist(fullfile(fld, [name '.slx']), 'file') == 2;
end
end

function s = JS_or_P(JS, P, field)
if ~isempty(JS)
    s = JS.(field);
else
    s = P.gains_baseline;
end
end

function [dmax, worst] = maxdiff(a, b, fields)
% NaN-aware max |a.f - b.f| over the fields (Inf if sizes or NaN patterns differ)
dmax = 0;
worst = '';
for i = 1:numel(fields)
    x = double(a.(fields{i})(:));
    y = double(b.(fields{i})(:));
    if numel(x) ~= numel(y)
        d = Inf;
    else
        nx = isnan(x);
        if any(nx ~= isnan(y))
            d = Inf;
        else
            d = max([0; abs(x(~nx) - y(~nx))]);
        end
    end
    if d > dmax || (isempty(worst) && d == dmax && d > 0)
        dmax = d;
        worst = fields{i};
    end
end
end

function tf = same_fields(a, b, fields)
tf = true;
for i = 1:numel(fields)
    tf = tf && isequaln(a.(fields{i}), b.(fields{i}));
end
end

function s = diffstr(d, f)
if d == 0
    s = 'max|diff| 0 (bit-exact)';
elseif isempty(f)
    s = sprintf('max|diff| %.3g', d);
else
    s = sprintf('max|diff| %.3g (%s)', d, f);
end
end

function rows = add(rows, name, ok, detail)
rows(end + 1, :) = {name, logical(ok), detail};
end

function tf = any_fail(rows, prefix)
tf = false;
for i = 1:size(rows, 1)
    if strncmp(rows{i, 1}, prefix, numel(prefix)) && ~rows{i, 2}
        tf = true;
    end
end
end

function v = col(h, M, name)
j = find(strcmp(h, name));
if numel(j) ~= 1
    error('check_simulink_models:col', 'column %s not found', name);
end
v = M(:, j);
end

function s = mcfg(s, mfold, name)
% simcfg for one model (folder only needed when it is not loaded / on the path)
s.model = name;
if ~isempty(mfold.(name))
    s.folder = mfold.(name);
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
