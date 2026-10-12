function V = emu_model_vars(ctrl, gains, P, opts)
%EMU_MODEL_VARS  All workspace variables of the Simulink models for one run.
%   V = emu_model_vars(ctrl, gains, P, opts)   same arguments as sim_boost
%   Returns a struct with the values to assign in the model workspace (or
%   pass with SimulationInput.setVariable) and the run settings:
%     model   'boost_avg' (opts.plant 'avg') | 'boost_cycle' ('switched')
%     CP, X0  controller block Parameter and Unit Delay 'xc' initial state
%             (make_ctrl_params)
%     LP, VP  scenario block Parameters (make_profile_vec)
%     PP      plant block Parameter (make_plant_params(Pplant, opts))
%     XP0     Unit Delay 'xp' initial state: [iL0; Vo0] or [iL0; Vo0; Vo0]
%     Ts, N   sample time and number of samples, N = fix(T_end/Ts);
%             StopTime = (N-1)*Ts gives N samples
%     t       (0:N-1)'*Ts;  noise  4 x N root-Inport data (emu_noise)
%     has_noise  true when any noise channel is active
%     name, lspec, vspec, plant, pwm, sd, seed   resolved options
%   Used by emulate_simulink, so the emulation and a Simulink run that
%   assigns these variables start from identical parameters.
if nargin < 2
    gains = [];
end
if nargin < 3 || isempty(P)
    P = boost_params();
end
if nargin < 4 || isempty(opts)
    opts = struct();
end

[V.CP, V.X0, V.name] = make_ctrl_params(ctrl, gains, P, opts);
V.Ts = P.Ts;
T_end = getopt(opts, 'T_end', P.T_end);
V.N = fix(T_end / V.Ts);
V.t = (0:V.N - 1)' * V.Ts;

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
V.lspec = lspec;
V.vspec = vspec;
V.LP = make_profile_vec(lspec, 'load', P);
V.VP = make_profile_vec(vspec, 'vin', P);

plant = lower(getopt(opts, 'plant', 'avg'));
switched = strcmp(plant, 'switched');
if ~switched && ~strcmp(plant, 'avg')
    error('emulate_simulink:plant', 'plant must be ''avg'' or ''switched''');
end
V.plant = plant;
Pp = getopt(opts, 'Pplant', P);
if Pp.Ts ~= V.Ts
    error('emulate_simulink:Ts', 'Pplant.Ts must equal P.Ts (single-rate Simulink model)');
end
if ~switched && (getopt(Pp, 'rL', 0) ~= 0 || getopt(Pp, 'rC', 0) ~= 0 || ...
        getopt(Pp, 'Ron', 0) ~= 0 || getopt(Pp, 'VF', 0) ~= 0)
    warning('emulate_simulink:parasitics', ...
        'averaged plant ignores rL/rC/Ron/VF; use opts.plant = ''switched''');
end
V.PP = make_plant_params(Pp, opts);
if V.PP(10) ~= 0
    V.pwm = 'center';
    if ~switched
        warning('emulate_simulink:pwm', 'opts.pwm only affects the switched plant (opts.plant = ''switched'')');
    end
else
    V.pwm = 'trailing';
end
[V.noise, V.sd] = emu_noise(opts, V.N);
V.has_noise = any(V.sd > 0);
V.seed = getopt(opts, 'seed', 0);

iL0 = getopt(opts, 'iL0', 0.0);
Vo0 = getopt(opts, 'Vo0', P.Vin);
if switched
    V.XP0 = [iL0; Vo0; Vo0];
    V.model = 'boost_cycle';
else
    V.XP0 = [iL0; Vo0];
    V.model = 'boost_avg';
end
end

function v = getopt(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
