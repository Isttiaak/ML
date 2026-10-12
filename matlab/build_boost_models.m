function files = build_boost_models(which_models, folder, bopts)
%BUILD_BOOST_MODELS  Create the Simulink models boost_avg.slx and/or boost_cycle.slx.
%   files = build_boost_models(which_models, folder, bopts)
%     which_models  'avg' | 'cycle' | 'both' (default 'both')
%     folder        where the .slx files are saved (default pwd; created if
%                   missing).  Keep it on the MATLAB path together with the
%                   folder of blk_*.m (the MATLAB Function blocks call them).
%     bopts         optional struct:
%                     P       boost_params() struct for the default workspace
%                     ctrl    controller of the default workspace ('cascade_pi')
%                     update  compile check after building (true)
%                     open    open the models in the editor (false)
%   files: cellstr with the full paths of the saved models.
%
%   UNTESTED here (no MATLAB): written against the documented APIs only
%   (add_block / add_line / set_param, MATLABFunctionConfiguration, the
%   Stateflow API for the data scope).  Steps that may differ between
%   releases are wrapped in try/catch and print what to do by hand.  The
%   wiring was checked in Octave with a mock of these APIs that executes
%   the recorded block diagram and reproduces sim_boost.
%
%   Model boost_avg (single rate Ts, fixed-step discrete, one sample per Ts):
%     k          Unit Delay, IC 0                         sample counter
%     Scenario   MATLAB Function [R, Vin, k_next] = blk_scenario(k, LP, VP, Ts)
%     xp         Unit Delay, IC XP0 = [iL0; Vo0]          plant state
%     xpDemux    Demux (2)                                iL = xp(1), Vo = xp(2)
%     io         Divide  Vo / R
%     truth      Mux (5) [Vo; iL; io; Vin; R]
%     noise      root Inport, width 4, sample time Ts     [nVo; niL; nio; nVin]
%     Controller MATLAB Function [d, IL_ref, xc_next, meas] =
%                                  blk_controller(truth, noise, xc, CP)
%     xc         Unit Delay, IC X0 (8x1)                  controller state
%     Plant      MATLAB Function xp_next = blk_plant_avg(xp, d, R, Vin, PP)
%     ylogMux    Mux (6) [Vo; iL; d; IL_ref; R; Vin]  ->  To Workspace 'ylog'
%     ymeas      To Workspace 'ymeas' <- meas (4)
%   Model boost_cycle: same, but
%     xp         IC XP0 = [iL0; Vo0; Vo0] = [iL; vC; Vout]; xpDemux (3), Vo = xp(3)
%     Plant      [xp_next, env] = blk_plant_switched(xp, d, R, Vin, PP)
%     yenv       To Workspace 'yenv' <- env (7)
%     yvc        To Workspace 'yvc'  <- vC = xp(2)
%   All To Workspace blocks: SaveFormat 'Array', SampleTime Ts.
%   Block Parameters (Scope 'Parameter', tunable) LP, VP, Ts, CP, PP and the
%   Unit Delay initial conditions X0, XP0 live in the MODEL workspace
%   (setup_boost_workspace), together with N for StopTime '(N-1)*Ts'.
%   Solver: Fixed-step, FixedStepDiscrete, FixedStep Ts, single-tasking,
%   algebraic loops = error, single simulation output (out.ylog ...).

if nargin < 1 || isempty(which_models)
    which_models = 'both';
end
if nargin < 2 || isempty(folder)
    folder = pwd;
end
if nargin < 3 || isempty(bopts)
    bopts = struct();
end
if isempty(which('new_system'))
    error('build_boost_models:simulink', 'Simulink is required (new_system not found)');
end
if ~isfield(bopts, 'P') || isempty(bopts.P)
    bopts.P = boost_params();
end
if ~isfield(bopts, 'ctrl') || isempty(bopts.ctrl)
    bopts.ctrl = 'cascade_pi';
end
if ~isfield(bopts, 'update') || isempty(bopts.update)
    bopts.update = true;
end
if ~isfield(bopts, 'open') || isempty(bopts.open)
    bopts.open = false;
end
switch lower(which_models)
    case 'avg'
        list = {'boost_avg'};
    case 'cycle'
        list = {'boost_cycle'};
    case 'both'
        list = {'boost_avg', 'boost_cycle'};
    otherwise
        error('build_boost_models:which', 'which_models must be ''avg'', ''cycle'' or ''both''');
end
if exist(folder, 'dir') ~= 7
    mkdir(folder);
end
here = pwd;                        % absolute folder (for the path checks)
cd(folder);
folder = pwd;
cd(here);
if isempty(which('blk_controller'))
    warning('build_boost_models:path', ...
        'blk_controller.m is not on the MATLAB path; add the folder of the blk_*.m files before simulating');
end

files = cell(1, numel(list));
for i = 1:numel(list)
    files{i} = build_one(list{i}, strcmp(list{i}, 'boost_cycle'), folder, bopts);
end
end

% ========================================================================
function file = build_one(mdl, cycle, folder, bopts)
fprintf('\n=== building %s (%s plant) ===\n', mdl, ternary(cycle, 'switched', 'averaged'));
file = fullfile(folder, [mdl '.slx']);
if bdIsLoaded(mdl)
    dirty = 'off';
    try
        dirty = get_param(mdl, 'Dirty');
    catch
    end
    if strcmp(dirty, 'on')
        error('build_boost_models:dirty', ...
            '%s is open with unsaved changes; save or close it before rebuilding', mdl);
    end
    close_system(mdl, 0);          % the loaded (saved) copy is replaced by the rebuilt model
end
other = which([mdl '.slx']);
if ~isempty(other) && ~strcmp(other, file)
    warning('build_boost_models:shadow', '%s is also on the path (%s); the new model is saved to %s', ...
        [mdl '.slx'], other, file);
end
new_system(mdl);
nfail = 0;

% ---- blocks (Position = [left top right bottom])
add_block('simulink/Discrete/Unit Delay', [mdl '/k'], ...
    'InitialCondition', '0', 'SampleTime', 'Ts', 'Position', [40 45 80 85]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Scenario'], ...
    'Position', [140 30 300 130]);
add_block('simulink/Discrete/Unit Delay', [mdl '/xp'], ...
    'InitialCondition', 'XP0', 'SampleTime', 'Ts', 'Position', [40 200 80 240]);
if cycle
    add_block('simulink/Signal Routing/Demux', [mdl '/xpDemux'], ...
        'Outputs', '3', 'Position', [140 180 145 260]);
else
    add_block('simulink/Signal Routing/Demux', [mdl '/xpDemux'], ...
        'Outputs', '2', 'Position', [140 190 145 250]);
end
add_block('simulink/Math Operations/Divide', [mdl '/io'], ...
    'Position', [380 160 410 190]);                       % default inputs '*/': in1 / in2
add_block('simulink/Signal Routing/Mux', [mdl '/truth'], ...
    'Inputs', '5', 'Position', [470 60 475 260]);
add_block('simulink/Sources/In1', [mdl '/noise'], ...
    'PortDimensions', '4', 'SampleTime', 'Ts', 'Position', [470 300 500 314]);
add_block('simulink/Discrete/Unit Delay', [mdl '/xc'], ...
    'InitialCondition', 'X0', 'SampleTime', 'Ts', 'Position', [470 360 510 400]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Controller'], ...
    'Position', [580 120 760 380]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Plant'], ...
    'Position', [880 160 1040 300]);
add_block('simulink/Signal Routing/Mux', [mdl '/ylogMux'], ...
    'Inputs', '6', 'Position', [1120 40 1125 220]);
add_block('simulink/Sinks/To Workspace', [mdl '/ylog'], ...
    'VariableName', 'ylog', 'SaveFormat', 'Array', 'SampleTime', 'Ts', 'Position', [1180 115 1260 145]);
add_block('simulink/Sinks/To Workspace', [mdl '/ymeas'], ...
    'VariableName', 'ymeas', 'SaveFormat', 'Array', 'SampleTime', 'Ts', 'Position', [880 380 960 410]);
if cycle
    add_block('simulink/Sinks/To Workspace', [mdl '/yenv'], ...
        'VariableName', 'yenv', 'SaveFormat', 'Array', 'SampleTime', 'Ts', 'Position', [1120 260 1200 290]);
    add_block('simulink/Sinks/To Workspace', [mdl '/yvc'], ...
        'VariableName', 'yvc', 'SaveFormat', 'Array', 'SampleTime', 'Ts', 'Position', [300 260 380 290]);
end

% ---- MATLAB Function blocks: code first (creates the ports), then scopes
nl = sprintf('\n');
code_scen = ['function [R, Vin, k_next] = scen(k, LP, VP, Ts)' nl ...
    '[R, Vin, k_next] = blk_scenario(k, LP, VP, Ts);' nl 'end' nl];
code_ctrl = ['function [d, IL_ref, xc_next, meas] = ctrl(truth, noise, xc, CP)' nl ...
    '[d, IL_ref, xc_next, meas] = blk_controller(truth, noise, xc, CP);' nl 'end' nl];
if cycle
    code_plant = ['function [xp_next, env] = plant(xp, d, R, Vin, PP)' nl ...
        '[xp_next, env] = blk_plant_switched(xp, d, R, Vin, PP);' nl 'end' nl];
    xp_size = '[3 1]';
else
    code_plant = ['function xp_next = plant(xp, d, R, Vin, PP)' nl ...
        'xp_next = blk_plant_avg(xp, d, R, Vin, PP);' nl 'end' nl];
    xp_size = '[2 1]';
end
% fixed sizes for the inputs that close a Unit Delay loop, so that size
% propagation does not depend on the loop (the other inputs inherit)
nfail = nfail + setup_mfb([mdl '/Scenario'], code_scen, {'LP', 'VP', 'Ts'}, {'k', '1'});
nfail = nfail + setup_mfb([mdl '/Controller'], code_ctrl, {'CP'}, {'xc', '[8 1]'});
nfail = nfail + setup_mfb([mdl '/Plant'], code_plant, {'PP'}, {'xp', xp_size});

% ---- signal lines ('Block/port'; a second line from the same output port
%      becomes a branch)
if cycle
    vo_src = 'xpDemux/3';
else
    vo_src = 'xpDemux/2';
end
L = { ...
    'k/1', 'Scenario/1'; ...          % k
    'Scenario/3', 'k/1'; ...          % k_next -> counter
    'xp/1', 'xpDemux/1'; ...
    'xp/1', 'Plant/1'; ...            % xp
    vo_src, 'io/1'; ...               % Vo / R
    'Scenario/1', 'io/2'; ...
    vo_src, 'truth/1'; ...            % truth = [Vo; iL; io; Vin; R]
    'xpDemux/1', 'truth/2'; ...
    'io/1', 'truth/3'; ...
    'Scenario/2', 'truth/4'; ...
    'Scenario/1', 'truth/5'; ...
    'truth/1', 'Controller/1'; ...
    'noise/1', 'Controller/2'; ...
    'xc/1', 'Controller/3'; ...
    'Controller/3', 'xc/1'; ...       % xc_next -> controller state
    'Controller/1', 'Plant/2'; ...    % d
    'Scenario/1', 'Plant/3'; ...      % R
    'Scenario/2', 'Plant/4'; ...      % Vin
    'Plant/1', 'xp/1'; ...            % xp_next -> plant state
    vo_src, 'ylogMux/1'; ...          % ylog = [Vo iL d IL_ref R Vin]
    'xpDemux/1', 'ylogMux/2'; ...
    'Controller/1', 'ylogMux/3'; ...
    'Controller/2', 'ylogMux/4'; ...
    'Scenario/1', 'ylogMux/5'; ...
    'Scenario/2', 'ylogMux/6'; ...
    'ylogMux/1', 'ylog/1'; ...
    'Controller/4', 'ymeas/1'};
if cycle
    L = [L; {'Plant/2', 'yenv/1'; 'xpDemux/2', 'yvc/1'}];
end
for i = 1:size(L, 1)
    try
        add_line(mdl, L{i, 1}, L{i, 2}, 'autorouting', 'on');
    catch err
        nfail = nfail + 1;
        fprintf('  MANUAL STEP: draw a line %s -> %s  (add_line failed: %s)\n', L{i, 1}, L{i, 2}, err.message);
    end
end

% ---- default model workspace (cascade_pi baseline, legacy scenario)
try
    if cycle
        plant = 'switched';
    else
        plant = 'avg';
    end
    S = setup_boost_workspace(mdl, bopts.ctrl, [], bopts.P, struct('plant', plant));
    fprintf('  model workspace: CP X0 LP VP PP XP0 Ts N assigned (%s, N = %d)\n', S.name, S.N);
catch err
    nfail = nfail + 1;
    fprintf('  MANUAL STEP: setup_boost_workspace(''%s'') failed: %s\n', mdl, err.message);
end

% ---- solver / data import-export
cfg = {'SolverType', 'Fixed-step'; 'Solver', 'FixedStepDiscrete'; 'FixedStep', 'Ts'; ...
    'StartTime', '0'; 'StopTime', '(N-1)*Ts'; 'EnableMultiTasking', 'off'; ...
    'AlgebraicLoopMsg', 'error'; 'ReturnWorkspaceOutputs', 'on'};
try
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', 'Ts', ...
        'StartTime', '0', 'StopTime', '(N-1)*Ts', 'EnableMultiTasking', 'off', ...
        'AlgebraicLoopMsg', 'error', 'ReturnWorkspaceOutputs', 'on');
catch
    for i = 1:size(cfg, 1)
        try
            set_param(mdl, cfg{i, 1}, cfg{i, 2});
        catch err
            nfail = nfail + 1;
            fprintf('  MANUAL STEP: Model Settings: set %s = %s (%s)\n', cfg{i, 1}, cfg{i, 2}, err.message);
        end
    end
end

save_system(mdl, file);
fprintf('  saved %s\n', file);

if bopts.update
    try
        set_param(mdl, 'SimulationCommand', 'update');
        fprintf('  update diagram (Ctrl+D): OK\n');
    catch err
        nfail = nfail + 1;
        fprintf('  UPDATE DIAGRAM FAILED: %s\n', err.message);
        print_causes(err, '    ');
        fprintf(['  Check: blk_*.m on the path, every MATLAB Function argument CP/LP/VP/PP/Ts has\n' ...
            '  Scope = Parameter, Size of xc = [8 1] and xp = %s, all lines connected.\n'], xp_size);
    end
end
if bopts.open
    open_system(mdl);
end
if nfail == 0
    fprintf('  %s: built without manual steps\n', mdl);
else
    fprintf('  %s: %d step(s) need manual attention (see MANUAL STEP lines), then save the model\n', mdl, nfail);
end
end

% ------------------------------------------------------------------------
function nfail = setup_mfb(blk, code, params, sizes)
% Function code, discrete update at Ts, Parameter scope and fixed sizes.
nfail = 0;
cfg_ok = false;
try
    cfg = get_param(blk, 'MATLABFunctionConfiguration');
    cfg.FunctionScript = code;
    cfg.UpdateMethod = 'Discrete';        % before SampleTime
    cfg.SampleTime = 'Ts';
    cfg_ok = true;
catch err
    fprintf('  note: MATLABFunctionConfiguration failed for %s (%s); trying the Stateflow API\n', ...
        blk, err.message);
end
ch = [];
sf_msg = 'Stateflow chart of the block not found';
try
    rt = sfroot;
    ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
catch err
    sf_msg = ['Stateflow API not available: ' err.message];
end
if ~cfg_ok
    try
        ch.Script = code;
        fprintf(['  MANUAL STEP: %s: Property Inspector > Update method = Discrete, ' ...
            'Sample time = Ts\n'], blk);
        nfail = nfail + 1;
    catch
        fprintf(['  MANUAL STEP: open %s and replace its code by:\n%s' ...
            '  then set Update method = Discrete, Sample time = Ts\n'], blk, code);
        nfail = nfail + 1;
    end
end
for i = 1:numel(params)
    try
        if isempty(ch)
            error('build_boost_models:sf', '%s', sf_msg);
        end
        dd = ch.find('-isa', 'Stateflow.Data', 'Name', params{i});
        if isempty(dd)
            error('build_boost_models:data', 'data ''%s'' not found', params{i});
        end
        dd.Scope = 'Parameter';
    catch err
        nfail = nfail + 1;
        fprintf(['  MANUAL STEP: %s: Function tab > Edit Data (Symbols pane): set %s Scope = Parameter ' ...
            '(tunable)  [%s]\n'], blk, params{i}, err.message);
    end
end
for i = 1:size(sizes, 1)
    try
        if isempty(ch)
            error('build_boost_models:sf', '%s', sf_msg);
        end
        dd = ch.find('-isa', 'Stateflow.Data', 'Name', sizes{i, 1});
        if isempty(dd)
            error('build_boost_models:data', 'data ''%s'' not found', sizes{i, 1});
        end
        dd.Props.Array.Size = sizes{i, 2};
        dd.DataType = 'double';
    catch err
        % not fatal: inheritance usually resolves the sizes as well
        fprintf(['  note: %s: could not fix input %s to size %s / double (%s); if the update ' ...
            'diagram reports a size problem, set it in Edit Data\n'], blk, sizes{i, 1}, sizes{i, 2}, err.message);
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

function r = ternary(c, a, b)
if c
    r = a;
else
    r = b;
end
end
