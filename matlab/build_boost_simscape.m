function file = build_boost_simscape(folder, bopts)
%BUILD_BOOST_SIMSCAPE  Create boost_simscape.slx: Simscape power stage + the same controller.
%   file = build_boost_simscape()            saves boost_simscape.slx in pwd
%   file = build_boost_simscape(folder, bopts)
%     bopts.ctrl    controller for the default workspace ('cascade_pi')
%     bopts.update  compile check after building (true)
%     bopts.open    open the model (false)
%   Needs Simscape (Foundation library).  The power stage is the composite
%   component boost_stage.ssc (inductor, variable-resistor switch, diode,
%   capacitor, load, sensors; the diode is the Foundation Diode,
%   foundation.electrical.elements.pwl_diode), used through a Simscape
%   Component block.
%
%   Signal flow (controller blocks run at Ts, the circuit at the local
%   solver step h = Ts/500):
%     k / Scenario   counter and load / Vin profiles (as in boost_avg)
%     PowerStage     Simscape circuit; inputs Rsw, Rload, Vin; outputs iL, vo, io
%     zoh_*          samples iL, vo, io at t = k*Ts (centre of the ON pulse
%                    for the centre-aligned carrier = average current in CCM)
%     Controller     blk_controller (same code as boost_avg)
%     dly            Unit Delay (IC D0): one-sample DSP latency, breaks the loop
%     dsel           open-loop override (OL = 1 -> fixed duty d_ol)
%     carrier        triangle 0 -> cpk -> 0 over Ts (valley at k*Ts, shifted by
%                    h/2 so each solver step is gated at its midpoint)
%     gate           Rsw = Ron_sw while d > carrier, else Roff_sw
%   Logs: ylog [vo iL d IL_ref R Vin] and ymeas at Ts; yfine [iL vo] at h
%   (for the switching ripple).  Workspace: setup_boost_simscape.

if nargin < 1 || isempty(folder)
    folder = pwd;
end
if nargin < 2 || isempty(bopts)
    bopts = struct();
end
ctrl = getf(bopts, 'ctrl', 'cascade_pi');
do_update = getf(bopts, 'update', true);
do_open = getf(bopts, 'open', false);
P = boost_params();

mdl = 'boost_simscape';
if exist(folder, 'dir') ~= 7
    mkdir(folder);
end
here = pwd;
cd(folder);
folder = pwd;
cd(here);
file = fullfile(folder, [mdl '.slx']);
if bdIsLoaded(mdl)
    if strcmp(get_param(mdl, 'Dirty'), 'on')
        error('build_boost_simscape:dirty', '%s is open with unsaved changes; save or close it first', mdl);
    end
    close_system(mdl, 0);
end
if isempty(which('boost_stage.ssc')) && exist(fullfile(fileparts(mfilename('fullpath')), 'boost_stage.ssc'), 'file') ~= 2
    error('build_boost_simscape:ssc', 'boost_stage.ssc not found next to this file');
end
fprintf('\n=== building %s (Simscape power stage) ===\n', mdl);
load_system('simulink');
load_system('nesl_utility');
load_system('fl_lib');
new_system(mdl);
nfail = 0;

% ---- control part (same as boost_avg)
add_block('simulink/Discrete/Unit Delay', [mdl '/k'], ...
    'InitialCondition', '0', 'SampleTime', 'Ts', 'Position', [40 45 80 85]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Scenario'], ...
    'Position', [140 30 300 130]);
add_block('simulink/Signal Routing/Mux', [mdl '/truth'], ...
    'Inputs', '5', 'Position', [900 60 905 260]);
add_block('simulink/Sources/In1', [mdl '/noise'], ...
    'PortDimensions', '4', 'SampleTime', 'Ts', 'Position', [900 300 930 314]);
add_block('simulink/Discrete/Unit Delay', [mdl '/xc'], ...
    'InitialCondition', 'X0', 'SampleTime', 'Ts', 'Position', [900 360 940 400]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Controller'], ...
    'Position', [1000 120 1180 380]);
add_block('simulink/Discrete/Unit Delay', [mdl '/dly'], ...
    'InitialCondition', 'D0', 'SampleTime', 'Ts', 'Position', [1240 140 1280 180]);
add_block('simulink/Sources/Constant', [mdl '/d_ol'], ...
    'Value', 'd_ol', 'Position', [1240 60 1280 90]);
add_block('simulink/Sources/Constant', [mdl '/OL'], ...
    'Value', 'OL', 'Position', [1240 100 1280 130]);
add_block('simulink/Signal Routing/Switch', [mdl '/dsel'], ...
    'Criteria', 'u2 > Threshold', 'Threshold', '0.5', 'Position', [1330 60 1370 180]);

% ---- modulator: centre-aligned carrier and switch resistance
% triangle with its valley at k*Ts, shifted by half a solver step: the
% gate of each solver step is then decided at the step midpoint, so the ON
% time is d*Ts quantised to +/- h/2 without bias (comparing at the step
% instants counts both edges and gives one extra step per period, i.e.
% d + h/Ts, about +0.19 V on Vo at h = Ts/500)
add_block('simulink/Sources/Repeating Sequence', [mdl '/carrier'], ...
    'rep_seq_t', '[0 h/2 Ts/2+h/2 Ts]', 'rep_seq_y', '[cpk*h/Ts 0 cpk cpk*h/Ts]', ...
    'Position', [1330 230 1370 260]);
add_block('simulink/Math Operations/Sum', [mdl '/dmc'], ...
    'Inputs', '+-', 'Position', [1420 140 1450 170]);
add_block('simulink/Sources/Constant', [mdl '/Ron'], ...
    'Value', 'Ron_sw', 'Position', [1420 60 1460 90]);
add_block('simulink/Sources/Constant', [mdl '/Roff'], ...
    'Value', 'Roff_sw', 'Position', [1420 220 1460 250]);
add_block('simulink/Signal Routing/Switch', [mdl '/gate'], ...
    'Criteria', 'u2 > Threshold', 'Threshold', '0', 'Position', [1500 60 1540 250]);

% ---- Simscape power stage
add_block('nesl_utility/Simulink-PS Converter', [mdl '/S2P_sw'], 'Position', [1590 140 1620 170]);
add_block('nesl_utility/Simulink-PS Converter', [mdl '/S2P_R'], 'Position', [380 160 410 190]);
add_block('nesl_utility/Simulink-PS Converter', [mdl '/S2P_V'], 'Position', [380 220 410 250]);
comp = [mdl '/PowerStage'];
add_comp(comp, [450 120 650 300]);
add_block('nesl_utility/PS-Simulink Converter', [mdl '/P2S_iL'], 'Position', [700 130 730 160]);
add_block('nesl_utility/PS-Simulink Converter', [mdl '/P2S_vo'], 'Position', [700 180 730 210]);
add_block('nesl_utility/PS-Simulink Converter', [mdl '/P2S_io'], 'Position', [700 230 730 260]);
add_block('nesl_utility/Solver Configuration', [mdl '/Solver'], 'Position', [700 300 760 330]);
add_block('simulink/Discrete/Zero-Order Hold', [mdl '/zoh_iL'], 'SampleTime', 'Ts', 'Position', [780 130 810 160]);
add_block('simulink/Discrete/Zero-Order Hold', [mdl '/zoh_vo'], 'SampleTime', 'Ts', 'Position', [780 180 810 210]);
add_block('simulink/Discrete/Zero-Order Hold', [mdl '/zoh_io'], 'SampleTime', 'Ts', 'Position', [780 230 810 260]);

% ---- logging
add_block('simulink/Signal Routing/Mux', [mdl '/ylogMux'], 'Inputs', '6', 'Position', [1240 420 1245 600]);
add_block('simulink/Sinks/To Workspace', [mdl '/ylog'], ...
    'VariableName', 'ylog', 'SaveFormat', 'Array', 'SampleTime', 'Ts', 'Position', [1300 495 1380 525]);
add_block('simulink/Sinks/To Workspace', [mdl '/ymeas'], ...
    'VariableName', 'ymeas', 'SaveFormat', 'Array', 'SampleTime', 'Ts', 'Position', [1240 640 1320 670]);
add_block('simulink/Signal Routing/Mux', [mdl '/fineMux'], 'Inputs', '2', 'Position', [780 420 785 480]);
add_block('simulink/Sinks/To Workspace', [mdl '/yfine'], ...
    'VariableName', 'yfine', 'SaveFormat', 'Array', 'SampleTime', '-1', 'Decimation', 'fine_dec', ...
    'Position', [830 435 910 465]);

% ---- block parameters
try
    set_param([mdl '/S2P_sw'], 'Unit', 'Ohm');
    set_param([mdl '/S2P_R'], 'Unit', 'Ohm');
    set_param([mdl '/S2P_V'], 'Unit', 'V');
    set_param([mdl '/P2S_iL'], 'Unit', 'A');
    set_param([mdl '/P2S_vo'], 'Unit', 'V');
    set_param([mdl '/P2S_io'], 'Unit', 'A');
catch err
    nfail = nfail + 1;
    fprintf('  MANUAL STEP: set the converter units (Ohm, Ohm, V; A, V, A): %s\n', err.message);
end
cp = {'L', 'Lp'; 'rL', 'rLp'; 'C', 'Cp'; 'rC', 'rCp'; 'Vf', 'Vfp'; 'Rd', 'Rdp'; 'Gd', 'Gdp'; ...
    'iL0', 'iL0p'; 'vC0', 'vC0p'};
for i = 1:size(cp, 1)
    try
        set_param(comp, cp{i, 1}, cp{i, 2});
    catch err
        nfail = nfail + 1;
        fprintf('  MANUAL STEP: PowerStage parameter %s = %s (%s)\n', cp{i, 1}, cp{i, 2}, err.message);
    end
end
try
    % enough nonlinear / mode iterations per step so that the diode turns
    % off in the same step as the switch turns on (default 3 / 2)
    set_param([mdl '/Solver'], 'UseLocalSolver', 'on', 'LocalSolverChoice', 'NE_BACKWARD_EULER_ADVANCER', ...
        'LocalSolverSampleTime', 'h', 'DoDC', 'off', 'MaxNonlinIter', '20', 'MaxModeIter', '20');
catch err
    nfail = nfail + 1;
    fprintf('  MANUAL STEP: Solver Configuration: local solver Backward Euler, sample time h (%s)\n', err.message);
end

% ---- MATLAB Function blocks
nl = sprintf('\n');
code_scen = ['function [R, Vin, k_next] = scen(k, LP, VP, Ts)' nl ...
    '[R, Vin, k_next] = blk_scenario(k, LP, VP, Ts);' nl 'end' nl];
code_ctrl = ['function [d, IL_ref, xc_next, meas] = ctrl(truth, noise, xc, CP)' nl ...
    '[d, IL_ref, xc_next, meas] = blk_controller(truth, noise, xc, CP);' nl 'end' nl];
nfail = nfail + setup_mfb([mdl '/Scenario'], code_scen, {'LP', 'VP', 'Ts'}, {'k', '1'});
nfail = nfail + setup_mfb([mdl '/Controller'], code_ctrl, {'CP'}, {'xc', '[8 1]'});

% ---- Simulink lines
L = { ...
    'k/1', 'Scenario/1'; 'Scenario/3', 'k/1'; ...
    'Scenario/1', 'S2P_R/1'; 'Scenario/2', 'S2P_V/1'; ...
    'P2S_iL/1', 'zoh_iL/1'; 'P2S_vo/1', 'zoh_vo/1'; 'P2S_io/1', 'zoh_io/1'; ...
    'zoh_vo/1', 'truth/1'; 'zoh_iL/1', 'truth/2'; 'zoh_io/1', 'truth/3'; ...
    'Scenario/2', 'truth/4'; 'Scenario/1', 'truth/5'; ...
    'truth/1', 'Controller/1'; 'noise/1', 'Controller/2'; 'xc/1', 'Controller/3'; ...
    'Controller/3', 'xc/1'; 'Controller/1', 'dly/1'; ...
    'd_ol/1', 'dsel/1'; 'OL/1', 'dsel/2'; 'dly/1', 'dsel/3'; ...
    'dsel/1', 'dmc/1'; 'carrier/1', 'dmc/2'; ...
    'Ron/1', 'gate/1'; 'dmc/1', 'gate/2'; 'Roff/1', 'gate/3'; ...
    'gate/1', 'S2P_sw/1'; ...
    'zoh_vo/1', 'ylogMux/1'; 'zoh_iL/1', 'ylogMux/2'; 'dsel/1', 'ylogMux/3'; ...
    'Controller/2', 'ylogMux/4'; 'Scenario/1', 'ylogMux/5'; 'Scenario/2', 'ylogMux/6'; ...
    'ylogMux/1', 'ylog/1'; 'Controller/4', 'ymeas/1'; ...
    'P2S_iL/1', 'fineMux/1'; 'P2S_vo/1', 'fineMux/2'; 'fineMux/1', 'yfine/1'};
for i = 1:size(L, 1)
    try
        add_line(mdl, L{i, 1}, L{i, 2}, 'autorouting', 'on');
    catch err
        nfail = nfail + 1;
        fprintf('  MANUAL STEP: draw a line %s -> %s  (add_line failed: %s)\n', L{i, 1}, L{i, 2}, err.message);
    end
end

% ---- physical connections: converters <-> PowerStage <-> Solver Configuration
pc = get_param(comp, 'PortHandles');
fprintf('  PowerStage ports: %d left (LConn), %d right (RConn)\n', numel(pc.LConn), numel(pc.RConn));
print_ports(comp);
% boost_stage.ssc: inputs Rsw, Rload, Vin on the left (declaration order);
% outputs iL, vo, io and node G on the right
phys = { ...
    [mdl '/S2P_sw'], 'RConn', 1, comp, 'LConn', 1; ...
    [mdl '/S2P_R'],  'RConn', 1, comp, 'LConn', 2; ...
    [mdl '/S2P_V'],  'RConn', 1, comp, 'LConn', 3; ...
    comp, 'RConn', 1, [mdl '/P2S_iL'], 'LConn', 1; ...
    comp, 'RConn', 2, [mdl '/P2S_vo'], 'LConn', 1; ...
    comp, 'RConn', 3, [mdl '/P2S_io'], 'LConn', 1; ...
    comp, 'RConn', 4, [mdl '/Solver'], 'RConn', 1};
for i = 1:size(phys, 1)
    try
        h1 = get_param(phys{i, 1}, 'PortHandles');
        h2 = get_param(phys{i, 4}, 'PortHandles');
        add_line(mdl, h1.(phys{i, 2})(phys{i, 3}), h2.(phys{i, 5})(phys{i, 6}), 'autorouting', 'on');
    catch err
        nfail = nfail + 1;
        fprintf('  MANUAL STEP: connect %s %s%d -> %s %s%d (%s)\n', phys{i, 1}, phys{i, 2}, phys{i, 3}, ...
            phys{i, 4}, phys{i, 5}, phys{i, 6}, err.message);
    end
end

% ---- workspace and model settings
try
    S = setup_boost_simscape(mdl, ctrl, [], P, struct());
    fprintf('  model workspace assigned (%s, N = %d, h = %g)\n', S.name, S.N, S.h);
catch err
    nfail = nfail + 1;
    fprintf('  MANUAL STEP: setup_boost_simscape(''%s'') failed: %s\n', mdl, err.message);
end
cfg = {'SolverType', 'Fixed-step'; 'Solver', 'FixedStepDiscrete'; 'FixedStep', 'h'; ...
    'StartTime', '0'; 'StopTime', '(N-1)*Ts'; 'EnableMultiTasking', 'off'; ...
    'AlgebraicLoopMsg', 'error'; 'ReturnWorkspaceOutputs', 'on'; 'SingleTaskRateTransMsg', 'none'};
for i = 1:size(cfg, 1)
    try
        set_param(mdl, cfg{i, 1}, cfg{i, 2});
    catch err
        nfail = nfail + 1;
        fprintf('  MANUAL STEP: Model Settings: set %s = %s (%s)\n', cfg{i, 1}, cfg{i, 2}, err.message);
    end
end
save_system(mdl, file);
fprintf('  saved %s\n', file);
if do_update
    try
        set_param(mdl, 'SimulationCommand', 'update');
        fprintf('  update diagram (Ctrl+D): OK\n');
    catch err
        nfail = nfail + 1;
        fprintf('  UPDATE DIAGRAM FAILED: %s\n', err.message);
        print_causes(err, '    ');
    end
end
if do_open
    open_system(mdl);
end
if nfail == 0
    fprintf('  %s: built without manual steps\n', mdl);
else
    fprintf('  %s: %d step(s) need manual attention (see MANUAL STEP lines)\n', mdl, nfail);
    error('build_boost_simscape:incomplete', '%s: %d build step(s) failed', mdl, nfail);
end
end

% ------------------------------------------------------------------------
function add_comp(blk, pos)
% Simscape Component block referencing boost_stage.ssc
lib = {'nesl_utility/Simscape Component', 'nesl_utility/Simscape Component Block'};
ok = false;
msg = '';
for i = 1:numel(lib)
    try
        add_block(lib{i}, blk, 'Position', pos);
        ok = true;
        break;
    catch err
        msg = err.message;
    end
end
if ~ok
    error('build_boost_simscape:component', 'could not add a Simscape Component block (%s)', msg);
end
names = {'ComponentPath', 'SourceFile'};
for i = 1:numel(names)
    try
        set_param(blk, names{i}, 'boost_stage');
        fprintf('  PowerStage: %s = boost_stage\n', names{i});
        return;
    catch err
        msg = err.message;
        fprintf('  set_param(%s) failed: %s\n', names{i}, err.message);
        print_causes(err, '    ');
    end
end
error('build_boost_simscape:component', 'could not point the Simscape Component block to boost_stage (%s)', msg);
end

function print_ports(blk)
try
    pcon = get_param(blk, 'PortConnectivity');
    for i = 1:numel(pcon)
        fprintf('    port %-8s at [%g %g]\n', pcon(i).Type, pcon(i).Position(1), pcon(i).Position(2));
    end
catch
end
end

function nfail = setup_mfb(blk, code, params, sizes)
nfail = 0;
try
    cfg = get_param(blk, 'MATLABFunctionConfiguration');
    cfg.FunctionScript = code;
    cfg.UpdateMethod = 'Discrete';
    cfg.SampleTime = 'Ts';
catch err
    nfail = nfail + 1;
    fprintf('  MANUAL STEP: %s: paste the code and set Update method Discrete, Sample time Ts (%s)\n', ...
        blk, err.message);
    return;
end
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
for i = 1:numel(params)
    try
        dd = ch.find('-isa', 'Stateflow.Data', 'Name', params{i});
        dd.Scope = 'Parameter';
    catch err
        nfail = nfail + 1;
        fprintf('  MANUAL STEP: %s: set %s Scope = Parameter (%s)\n', blk, params{i}, err.message);
    end
end
for i = 1:size(sizes, 1)
    try
        dd = ch.find('-isa', 'Stateflow.Data', 'Name', sizes{i, 1});
        dd.Props.Array.Size = sizes{i, 2};
        dd.DataType = 'double';
    catch err
        fprintf('  note: %s: size of %s not fixed (%s)\n', blk, sizes{i, 1}, err.message);
    end
end
end

function print_causes(err, pre)
try
    c = err.cause;
    for i = 1:numel(c)
        fprintf('%scause: %s\n', pre, c{i}.message);
        print_causes(c{i}, [pre '  ']);
    end
catch
end
end

function v = getf(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
