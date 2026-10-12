function S = setup_boost_workspace(mdl, ctrl, gains, P, opts)
%SETUP_BOOST_WORKSPACE  Workspace variables of the Simulink models boost_avg / boost_cycle.
%   S = setup_boost_workspace(mdl, ctrl, gains, P, opts)
%     mdl    model name ('boost_avg', 'boost_cycle', or a hand-built model
%            with the same block parameters).  '' = do not touch Simulink,
%            only return the struct (this path also runs in Octave).
%     ctrl, gains, P, opts   exactly as for sim_boost (same defaults);
%            ctrl defaults to 'cascade_pi', gains [] = P.gains_baseline.
%   Computes and, when mdl is given, assigns in the MODEL workspace
%   (get_param(mdl,'ModelWorkspace')):
%     CP  (1x32) and X0 (8x1)   make_ctrl_params(ctrl, gains, P, opts)
%     LP, VP (1x7)              make_profile_vec(opts.load/'load'), (opts.vin/'vin')
%                               with the sim_boost defaults (legacy load step,
%                               constant Vin = P.Vin)
%     PP  (1x10)                make_plant_params(opts.Pplant (default P), opts)
%     XP0                       [iL0; Vo0] (boost_avg) or [iL0; Vo0; Vo0]
%                               (boost_cycle); iL0 = 0, Vo0 = P.Vin by default
%     Ts, N                     P.Ts and N = fix(T_end/Ts) (T_end = opts.T_end
%                               or P.T_end); the model StopTime '(N-1)*Ts'
%                               then gives N samples like sim_boost
%   Returned struct S: the variables above plus
%     model      'boost_avg' | 'boost_cycle' (plant kind; = mdl if given)
%     name       canonical controller name     plant 'avg' | 'switched'
%     pwm        'trailing' | 'center'         StopTime numeric string of (N-1)*Ts
%     t          (0:N-1)'*Ts
%     noise      4 x N sensor noise for the root Inport 'noise', drawn exactly
%                like sim_boost: rng(seed,'twister'), randn(n_active, N), caller
%                RNG state restored, rows Vo, iL, io, Vin scaled by the 1-sigma
%                values; zero rows for inactive channels
%     sd, seed, has_noise, lspec, vspec, assigned (true if written to mdl)
%   The plant kind comes from opts.plant ('avg' default / 'switched'); when
%   mdl is given and opts.plant is missing it is taken from the model
%   (boost_cycle has a To Workspace block 'yenv').  A mismatch is an error
%   because XP0 has 2 or 3 elements.
%   Noise is NOT stored in the model workspace: it enters through the root
%   Inport 'noise' (sim_boost_simulink passes S.noise with setExternalInput).
%   Simulink part UNTESTED here (no MATLAB); the mdl = '' path is tested in
%   Octave against emu_model_vars and sim_boost.

if nargin < 1 || isempty(mdl)
    mdl = '';
end
if nargin < 2 || isempty(ctrl)
    ctrl = 'cascade_pi';
end
if nargin < 3
    gains = [];
end
if nargin < 4 || isempty(P)
    P = boost_params();
end
if nargin < 5 || isempty(opts)
    opts = struct();
end

[S.CP, S.X0, S.name] = make_ctrl_params(ctrl, gains, P, opts);

% ---- plant kind (avg / switched) and model name
plant = lower(getopt(opts, 'plant', ''));
if ~isempty(mdl)
    ensure_loaded(mdl);
    mdl_cycle = model_is_cycle(mdl);
    if isempty(plant)
        if mdl_cycle
            plant = 'switched';
        else
            plant = 'avg';
        end
    elseif strcmp(plant, 'switched') ~= mdl_cycle
        error('setup_boost_workspace:plant', ...
            'opts.plant = ''%s'' does not match model %s (use boost_avg for ''avg'', boost_cycle for ''switched'')', ...
            plant, mdl);
    end
end
if isempty(plant)
    plant = 'avg';
end
switch plant
    case 'avg'
        switched = false;
    case 'switched'
        switched = true;
    otherwise
        error('setup_boost_workspace:plant', 'plant must be ''avg'' or ''switched''');
end
S.plant = plant;
if ~isempty(mdl)
    S.model = mdl;
elseif switched
    S.model = 'boost_cycle';
else
    S.model = 'boost_avg';
end

% ---- time base
S.Ts = P.Ts;
T_end = getopt(opts, 'T_end', P.T_end);
S.N = fix(T_end / S.Ts);
if S.N < 1
    error('setup_boost_workspace:T_end', 'T_end = %g gives no samples', T_end);
end
S.t = (0:S.N - 1)' * S.Ts;
S.StopTime = sprintf('%.17g', (S.N - 1) * S.Ts);

% ---- scenario profiles (sim_boost defaults)
lspec = getopt(opts, 'load', []);
if isempty(lspec)
    lspec = struct('kind', 'step', 'R0', P.R0, 'R_new', P.R1, 't_step', P.t_step);
end
if ~isfield(lspec, 'R0')
    lspec.R0 = P.R0;
end
vspec = getopt(opts, 'vin', []);
if isempty(vspec)
    vspec = struct('kind', 'const');
end
if ~isfield(vspec, 'Vin')
    vspec.Vin = P.Vin;
end
S.lspec = lspec;
S.vspec = vspec;
S.LP = make_profile_vec(lspec, 'load', P);
S.VP = make_profile_vec(vspec, 'vin', P);

% ---- plant parameters
Pp = getopt(opts, 'Pplant', P);
if Pp.Ts ~= S.Ts
    error('setup_boost_workspace:Ts', ...
        'Pplant.Ts must equal P.Ts (the Simulink models run at one rate Ts)');
end
if ~switched && (getopt(Pp, 'rL', 0) ~= 0 || getopt(Pp, 'rC', 0) ~= 0 || ...
        getopt(Pp, 'Ron', 0) ~= 0 || getopt(Pp, 'VF', 0) ~= 0)
    warning('setup_boost_workspace:parasitics', ...
        'averaged plant ignores rL/rC/Ron/VF; use opts.plant = ''switched'' (boost_cycle)');
end
S.PP = make_plant_params(Pp, opts);
if S.PP(10) ~= 0
    S.pwm = 'center';
    if ~switched
        warning('setup_boost_workspace:pwm', ...
            'opts.pwm only affects the switched plant (opts.plant = ''switched'', boost_cycle)');
    end
else
    S.pwm = 'trailing';
end

% ---- initial plant state
iL0 = getopt(opts, 'iL0', 0.0);
Vo0 = getopt(opts, 'Vo0', P.Vin);
if switched
    S.XP0 = [iL0; Vo0; Vo0];
else
    S.XP0 = [iL0; Vo0];
end

% ---- sensor noise (same stream as sim_boost)
S.sd = [getopt(opts, 'noise_std_v', 0), getopt(opts, 'noise_std_i', 0), ...
        getopt(opts, 'noise_std_io', 0), getopt(opts, 'noise_std_vin', 0)];
S.seed = getopt(opts, 'seed', 0);
S.noise = zeros(4, S.N);
act = find(S.sd > 0);
if ~isempty(act)
    rng_saved = rng;
    rng(S.seed, 'twister');
    z = randn(numel(act), S.N);
    rng(rng_saved);
    for j = 1:numel(act)
        S.noise(act(j), :) = S.sd(act(j)) * z(j, :);
    end
end
S.has_noise = ~isempty(act);

% ---- write to the model workspace
S.assigned = false;
if ~isempty(mdl)
    mws = get_param(mdl, 'ModelWorkspace');
    names = {'CP', 'X0', 'LP', 'VP', 'PP', 'XP0', 'Ts', 'N'};
    for i = 1:numel(names)
        assignin(mws, names{i}, S.(names{i}));
    end
    S.assigned = true;
end
end

% ------------------------------------------------------------------------
function ensure_loaded(mdl)
if ~bdIsLoaded(mdl)
    load_system(mdl);
end
end

function tf = model_is_cycle(mdl)
% boost_cycle is recognised by its To Workspace block 'yenv'
try
    tf = ~isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'yenv'));
catch
    tf = ~isempty(strfind(lower(mdl), 'cycle'));
end
end

function v = getopt(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
