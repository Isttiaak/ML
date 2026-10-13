function outs = sim_boost_simulink(ctrl, G, P, opts, simcfg)
%SIM_BOOST_SIMULINK  Batch runs of the Simulink models boost_avg / boost_cycle.
%   outs = sim_boost_simulink(ctrl, G, P, opts, simcfg)
%     ctrl    controller name as in sim_boost, or a cellstr with one name per
%             run (all runs share one model, so they can differ in the
%             controller: only the values of CP change)
%     G       gains, one ROW per run (n x ngains), or a cell array of gain
%             vectors (one per run; [] = baseline).  [] = one baseline run.
%     P       boost_params() struct ([] = default), shared by all runs
%     opts    sim_boost options: one struct for all runs, or a struct array /
%             cell array with one element per run (e.g. different seeds,
%             loads, Pplant).  Inputs of length 1 are broadcast to n runs.
%     simcfg  optional struct:
%       model         'boost_avg' (default) or 'boost_cycle' (default when
%                     opts.plant = 'switched'), or a hand-built model with the
%                     same parameters and logs
%       folder        folder of the .slx if the model is not on the path
%       use_parsim    false (default) | true (Parallel Computing Toolbox)
%       fast_restart  true (default): compile once, then only the tunable
%                     parameters, initial states and the root input change
%       workspace     'model' (default): CP X0 LP VP PP XP0 are set with
%                     setVariable(..., 'Workspace', model);  'base': set in the
%                     global workspace (hand-built models without model-
%                     workspace variables; Ts and N are set too)
%       backend       'simulink' (default) | 'emulate': no Simulink, every run
%                     is computed by emulate_simulink and its traces are
%                     passed through the same log parser (Octave check of
%                     this function)
%       verbose       false
%   outs  1 x n struct array, sample k at t = (k-1)*Ts like sim_boost:
%     t iL Vo d IL_ref R Vin        from To Workspace 'ylog' [Vo iL d IL_ref R Vin]
%     Vo_meas iL_meas io_meas vin_meas   from 'ymeas' (what the controller saw)
%     IL_ff                         derived like sim_boost (oracle / measured /
%                                   none formula on the logged signals)
%     boost_cycle only: Vc (from 'yvc', NaN if not logged), Vo_min Vo_max
%       iL_min iL_max iL_avg Vo_avg dcm (from 'yenv'; NaN/false at k = N like
%       sim_boost, which does not step the plant after the last sample)
%     ok, error   run status and error text (setup error or ErrorMessage)
%     info        model, backend, ctrl, gains, CP X0 LP VP PP XP0, N, Ts,
%                 StopTime, plant, pwm, seed, sd, has_noise, load, vin
%   Each run sets CP X0 LP VP PP XP0 (setVariable), StopTime = (N-1)*Ts as a
%   numeric string, and the 4 x N sensor noise as external input
%   timeseries(noise.', t) on the root Inport 'noise'.  The noise is drawn
%   exactly like sim_boost (rng(seed,'twister'), randn(n_active, N), caller
%   RNG restored, channel order Vo iL io Vin), so a run equals sim_boost with
%   the same arguments (check_simulink_models verifies this).
%   With workspace 'model' the variables must already exist in the model
%   workspace (setVariable only overrides them); they are written from the
%   first run only when one is missing or has the wrong size, so a saved
%   model normally stays unmodified.  Before parsim a modified model is
%   saved, because the workers load the .slx file.
%   Noise source: a model with the root Inport 'noise' gets the noise as
%   external input; a model without root Inport (Inport replaced by a
%   MATLAB Function n = blk_noise_lookup(k, NZ)) gets NZ (4 x N) per run with
%   setVariable.  NZ's size follows N, so all runs of one Fast Restart
%   batch must then have the same T_end.
%   Fast Restart needs every run to use the same model, Ts and parameter
%   sizes (always true here); delay <= 3 and Pplant.Ts == P.Ts are required
%   by the block code.
%   Simulink path UNTESTED here (no MATLAB); backend 'emulate' is tested in
%   Octave.  Example:
%     outs = sim_boost_simulink('cascade_pi', [0.03 20; 0.05 1], [], struct('ff_mode', 'measured'));
%     plot(outs(1).t, [outs.Vo])

if nargin < 1 || isempty(ctrl)
    ctrl = 'cascade_pi';
end
if nargin < 2
    G = [];
end
if nargin < 3 || isempty(P)
    P = boost_params();
end
if nargin < 4 || isempty(opts)
    opts = struct();
end
if nargin < 5 || isempty(simcfg)
    simcfg = struct();
end
backend = lower(getcfg(simcfg, 'backend', 'simulink'));
if ~any(strcmp(backend, {'simulink', 'emulate'}))
    error('sim_boost_simulink:backend', 'simcfg.backend must be ''simulink'' or ''emulate''');
end
verbose = getcfg(simcfg, 'verbose', false);

% ---- runs: broadcast ctrl / gains / opts to n
if ischar(ctrl)
    cl = {ctrl};
elseif iscell(ctrl)
    cl = ctrl(:)';
else
    error('sim_boost_simulink:ctrl', 'ctrl must be a char array or a cellstr');
end
if iscell(G)
    gl = G(:)';
elseif isempty(G)
    gl = {[]};
else
    gl = num2cell(G, 2)';
end
if iscell(opts)
    ol = opts(:)';
elseif isstruct(opts)
    ol = num2cell(opts(:)');
else
    error('sim_boost_simulink:opts', 'opts must be a struct, struct array or cell array');
end
n = max([numel(cl), numel(gl), numel(ol)]);
cl = expand(cl, n, 'ctrl');
gl = expand(gl, n, 'G');
ol = expand(ol, n, 'opts');

% ---- per-run variables (no Simulink involved)
S = cell(1, n);
err0 = cell(1, n);
for i = 1:n
    err0{i} = '';
    try
        S{i} = setup_boost_workspace('', cl{i}, gl{i}, P, ol{i});
    catch e
        err0{i} = ['setup: ' e.message];
    end
end
mdl = getcfg(simcfg, 'model', '');
if isempty(mdl)
    mdl = 'boost_avg';
    for i = 1:n
        if isempty(err0{i})
            mdl = S{i}.model;
            break;
        end
    end
end

% ---- model kind
if strcmp(backend, 'simulink')
    if isempty(which('sim')) || isempty(which('load_system'))
        error('sim_boost_simulink:simulink', 'Simulink is required (or use simcfg.backend = ''emulate'')');
    end
    if ~bdIsLoaded(mdl)
        folder = getcfg(simcfg, 'folder', '');
        if ~isempty(folder)
            load_system(fullfile(folder, [mdl '.slx']));
        else
            load_system(mdl);
        end
    end
    cycle = model_is_cycle(mdl);
else
    cycle = ~isempty(strfind(lower(mdl), 'cycle'));
end
for i = 1:n
    if isempty(err0{i}) && (strcmp(S{i}.plant, 'switched') ~= cycle)
        err0{i} = sprintf('run needs opts.plant = ''%s'' but model %s has the %s plant', S{i}.plant, mdl, ...
            ternary(cycle, 'switched', 'averaged'));
    end
end

% ---- output template (identical fields for every run)
base = struct('t', [], 'iL', [], 'Vo', [], 'd', [], 'IL_ref', [], 'IL_ff', [], 'R', [], 'Vin', [], ...
    'Vo_meas', [], 'iL_meas', [], 'io_meas', [], 'vin_meas', []);
if cycle
    cf = {'Vc', 'Vo_min', 'Vo_max', 'iL_min', 'iL_max', 'iL_avg', 'Vo_avg', 'dcm'};
    for j = 1:numel(cf)
        base.(cf{j}) = [];
    end
end
base.ok = false;
base.error = '';
base.info = struct();
outs = repmat(base, 1, n);
for i = 1:n
    outs(i).error = err0{i};
    if ~isempty(S{i})
        outs(i).info = make_info(S{i}, mdl, backend);
    end
end
idx = find(cellfun(@isempty, err0));
if isempty(idx)
    return;
end

% ---- run
if strcmp(backend, 'emulate')
    for j = 1:numel(idx)
        i = idx(j);
        try
            e = emulate_simulink(cl{i}, gl{i}, P, ol{i});
            lg.ylog = [e.Vo, e.iL, e.d, e.IL_ref, e.R, e.Vin];
            M = [e.Vo_meas, e.iL_meas, e.io_meas, e.vin_meas];
            lg.ymeas = reshape(M.', 4, 1, []);        % Simulink '3-D array' layout of a 4x1 signal
            if cycle
                E = [e.Vo_min, e.Vo_max, e.iL_min, e.iL_max, e.iL_avg, e.Vo_avg, double(e.dcm)];
                lg.yenv = reshape(E.', 7, 1, []);
                lg.yvc = e.Vc;
            end
            outs(i) = fill_out(outs(i), lg, S{i}, cycle);
        catch err
            outs(i).error = ['emulate: ' err.message];
        end
    end
else
    ws = lower(getcfg(simcfg, 'workspace', 'model'));
    if ~any(strcmp(ws, {'model', 'base'}))
        error('sim_boost_simulink:workspace', 'simcfg.workspace must be ''model'' or ''base''');
    end
    i1 = idx(1);
    lookup = uses_noise_lookup(mdl);
    vars = {'CP', 'X0', 'LP', 'VP', 'PP', 'XP0'};
    if lookup
        vars{end + 1} = 'NZ';
    end
    if strcmp(ws, 'model')
        % setVariable(..., 'Workspace', mdl) only overrides variables that
        % already exist there.  Write the defaults only when something is
        % missing (or Ts / sizes differ), so the model normally stays clean:
        % parsim workers load the SAVED .slx, not the copy in memory.
        if needs_refill(mdl, [vars, {'Ts', 'N'}], S{i1})
            setup_boost_workspace(mdl, cl{i1}, gl{i1}, P, ol{i1});
        end
    else
        vars = [vars, {'Ts', 'N'}];
        % model-workspace variables shadow the global ones silently
        clash = {};
        try
            mws = get_param(mdl, 'ModelWorkspace');
            for v = 1:numel(vars)
                if hasVariable(mws, vars{v})
                    clash{end + 1} = vars{v}; %#ok<AGROW>
                end
            end
        catch
        end
        if ~isempty(clash)
            error('sim_boost_simulink:workspace', ['the model workspace of %s defines %s, which ' ...
                'would silently override the per-run values; use simcfg.workspace = ''model'''], ...
                mdl, strjoin(clash, ' '));
        end
    end
    m = numel(idx);
    for j = m:-1:1
        s = S{idx(j)};
        inj = Simulink.SimulationInput(mdl);
        for v = 1:numel(vars)
            if strcmp(ws, 'model')
                inj = inj.setVariable(vars{v}, s.(vars{v}), 'Workspace', mdl);
            else
                inj = inj.setVariable(vars{v}, s.(vars{v}));
            end
        end
        inj = inj.setModelParameter('StopTime', s.StopTime);
        if ~lookup
            % root Inport 'noise' (zeros when no external input is given)
            inj = inj.setExternalInput(timeseries(s.noise.', s.t));
        end
        in(j) = inj; %#ok<AGROW>
    end
    if getcfg(simcfg, 'fast_restart', true)
        fr = 'on';
    else
        fr = 'off';
    end
    try
        if getcfg(simcfg, 'use_parsim', false)
            if strcmp(get_param(mdl, 'Dirty'), 'on')
                save_system(mdl);   % workers load the saved .slx, not the in-memory model
            end
            [files, dirs] = block_files(mdl);
            so = parsim(in, 'UseFastRestart', fr, 'TransferBaseWorkspaceVariables', 'off', ...
                'AttachedFiles', files, 'SetupFcn', @() addpath(dirs{:}));
        else
            so = sim(in, 'UseFastRestart', fr, 'ShowProgress', 'off', 'StopOnError', 'off');
        end
    catch err
        % a scalar SimulationInput does not capture errors: report per run
        for j = 1:m
            outs(idx(j)).error = ['sim: ' err.message];
        end
        return;
    end
    for j = 1:m
        i = idx(j);
        em = '';
        try
            em = char(so(j).ErrorMessage);
        catch
        end
        if ~isempty(em)
            outs(i).error = em;
            continue;
        end
        try
            lg = struct();
            lg.ylog = so(j).get('ylog');
            lg.ymeas = so(j).get('ymeas');
            if cycle
                lg.yenv = so(j).get('yenv');
                try
                    lg.yvc = so(j).get('yvc');
                catch
                    lg.yvc = [];
                end
            end
            outs(i) = fill_out(outs(i), lg, S{idx(j)}, cycle);
        catch err
            outs(i).error = ['reading logs: ' err.message];
        end
    end
end
if verbose
    nok = sum([outs.ok]);
    fprintf('sim_boost_simulink: %d/%d runs OK (%s, backend %s)\n', nok, n, mdl, backend);
    for i = find(~[outs.ok])
        fprintf('  run %d failed: %s\n', i, outs(i).error);
    end
end
end

% ========================================================================
function o = fill_out(o, lg, s, cycle)
N = s.N;
Y = to_rows(lg.ylog, 6, N, 'ylog');
o.t = s.t;
o.Vo = Y(:, 1);
o.iL = Y(:, 2);
o.d = Y(:, 3);
o.IL_ref = Y(:, 4);
o.R = Y(:, 5);
o.Vin = Y(:, 6);
M = to_rows(lg.ymeas, 4, N, 'ymeas');
o.Vo_meas = M(:, 1);
o.iL_meas = M(:, 2);
o.io_meas = M(:, 3);
o.vin_meas = M(:, 4);
cid = s.CP(1);
fid = s.CP(6);
Vref = s.CP(12);
if cid == 5
    o.IL_ff = nan(N, 1);
elseif fid == 1
    o.IL_ff = (Vref * Vref) ./ (o.Vin .* o.R);
elseif fid == 2
    o.IL_ff = Vref * o.io_meas ./ max(o.vin_meas, 1e-3);
else
    o.IL_ff = zeros(N, 1);
end
if cycle
    E = to_rows(lg.yenv, 7, N, 'yenv');
    E(N, :) = NaN;
    o.Vo_min = E(:, 1);
    o.Vo_max = E(:, 2);
    o.iL_min = E(:, 3);
    o.iL_max = E(:, 4);
    o.iL_avg = E(:, 5);
    o.Vo_avg = E(:, 6);
    o.dcm = E(:, 7) == 1;
    if isfield(lg, 'yvc') && ~isempty(lg.yvc)
        o.Vc = to_rows(lg.yvc, 1, N, 'yvc');
    else
        o.Vc = nan(N, 1);
    end
end
o.ok = true;
o.error = '';
end

function X = to_rows(x, w, N, name)
% logged Array data -> N x w (accepts T x w, w x T, w x 1 x T, 1 x w x T)
if isempty(x)
    error('sim_boost_simulink:log', 'logged variable %s is empty or missing', name);
end
x = double(x);
if ndims(x) == 3
    X = reshape(x, [], size(x, 3)).';
elseif size(x, 2) ~= w && size(x, 1) == w
    X = x.';
else
    X = x;
end
if size(X, 2) ~= w
    error('sim_boost_simulink:log', '%s has %d columns, expected %d', name, size(X, 2), w);
end
if size(X, 1) < N
    error('sim_boost_simulink:log', '%s has %d samples, expected %d (StopTime = (N-1)*Ts?)', ...
        name, size(X, 1), N);
end
X = X(1:N, :);
end

function info = make_info(s, mdl, backend)
nreq = [1 2 4 2 2];
info.model = mdl;
info.backend = backend;
info.ctrl = s.name;
info.gains = s.CP(2:1 + nreq(s.CP(1)));
info.plant = s.plant;
info.pwm = s.pwm;
info.CP = s.CP;
info.X0 = s.X0;
info.LP = s.LP;
info.VP = s.VP;
info.PP = s.PP;
info.XP0 = s.XP0;
info.N = s.N;
info.Ts = s.Ts;
info.StopTime = s.StopTime;
info.seed = s.seed;
info.sd = s.sd;
info.has_noise = s.has_noise;
info.load = s.lspec;
info.vin = s.vspec;
end

function [files, dirs] = block_files(mdl)
% code needed by the MATLAB Function blocks (for parsim workers)
deps = {'blk_controller', 'blk_scenario', 'blk_profile_eval', 'blk_plant_avg', 'blk_plant_switched', ...
    'blk_noise_lookup', 'sense_adc', 'round_half_even', 'ctrl_outer_pi', 'ctrl_inner_pi', ...
    'ctrl_inner_pid', 'ctrl_smc', 'ctrl_single_pi', 'ctrl_fcsmpc', 'plant_avg_step', 'sw_boost_period'};
files = {};
for k = 1:numel(deps)
    p = which(deps{k});
    if ~isempty(p)
        files{end + 1} = p; %#ok<AGROW>
    end
end
dirs = {fileparts(which('blk_controller'))};
try
    mf = get_param(mdl, 'FileName');
    if ~isempty(mf)
        dirs{end + 1} = fileparts(mf);
    end
catch
end
end

function tf = model_is_cycle(mdl)
try
    tf = ~isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'yenv'));
catch
    tf = ~isempty(strfind(lower(mdl), 'cycle'));
end
end

function c = expand(c, n, what)
if numel(c) == 1
    c = repmat(c, 1, n);
elseif numel(c) ~= n
    error('sim_boost_simulink:runs', '%s has %d entries, expected 1 or %d', what, numel(c), n);
end
end

function tf = uses_noise_lookup(mdl)
% models without a root Inport take the noise from blk_noise_lookup(k, NZ)
try
    tf = isempty(find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Inport'));
catch
    tf = false;
end
end

function tf = needs_refill(mdl, vars, s)
% true if a model-workspace variable is missing or has the wrong size / Ts
tf = true;
try
    mws = get_param(mdl, 'ModelWorkspace');
    for v = 1:numel(vars)
        if ~hasVariable(mws, vars{v}) || ...
                ~isequal(size(getVariable(mws, vars{v})), size(s.(vars{v})))
            return;
        end
    end
    tf = ~isequal(getVariable(mws, 'Ts'), s.Ts);
catch
    tf = true;   % API not available: fall back to writing the defaults
end
end

function v = getcfg(s, name, default)
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
