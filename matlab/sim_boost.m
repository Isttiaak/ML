function out = sim_boost(ctrl, gains, P, opts)
%SIM_BOOST  Closed-loop boost-converter simulation (legacy + corrected options).
%   out = sim_boost(ctrl, gains, P, opts)
%
%   ctrl  : 'fcsmpc' | 'cascade_pi' | 'cascade_pid' | 'smc' | 'single_pi'
%   gains : row/column vector in the notebook order ([] = P.gains_baseline):
%             fcsmpc [lsw]   cascade_pi [Kp_i Ki_i]   cascade_pid [Kp_i Ki_i Kd_i Tf]
%             smc [K_s phi]  single_pi [Kp_v Ki_v]
%   P     : boost_params() struct ([] = default)
%   opts  : struct, every field optional.  Defaults reproduce the notebook
%           sim_* functions (cell 1) and simulate_controller (cell 12):
%     load        load_profile spec   (default legacy step R0 -> R1 at t_step)
%     vin         vin_profile spec    (default const P.Vin)
%     ff_mode     'oracle' (default, legacy: IL_ff = Vref^2/(Vin_true*R_true))
%                 'measured' (IL_ff = Vref*io_meas/Vin_meas, io = Vout/R and Vin
%                             through sense_adc)
%                 'none' (IL_ff = 0, the outer integrator must find the load)
%     KPV, KIV, IL_MAX   outer-loop overrides (default P.KPV, P.KIV, P.IL_MAX)
%     delay       0 (default) or n >= 1 samples of computation delay: the duty
%                 computed at sample k is applied at k+n; the first n periods
%                 use D_ss (u = 0 for FCS-MPC).
%     noise_std_v, noise_std_i, noise_std_io, noise_std_vin   1-sigma sensor
%                 noise [V, A, A, V] (default 0)
%     adc_bits    0 = no quantisation (default); applied to all sensed channels
%     Vo_adc_range, iL_adc_range, io_adc_range, vin_adc_range  (default from P;
%                 use P.iL_adc_range_legacy to reproduce the notebook's [0 12])
%     seed        noise seed (default 0).  rng(seed,'twister') is used and the
%                 caller's global RNG state is restored afterwards.  One randn
%                 per active channel (std > 0) per sample, in the fixed order
%                 Vo, iL, io, Vin.  (numpy's stream cannot be reproduced.)
%     plant       'avg' (default, legacy Euler averaged model) | 'switched'
%                 (plant_switched_step: PWM, ripple, parasitics rL rC Ron VF)
%     nsub        sub-steps per period for the switched plant (default 100)
%     pwm         switched plant only: 'trailing' (default) | 'center' ('centre')
%                 PWM alignment w.r.t. the sampling instant t = (k-1)*Ts
%                 (see plant_switched_step).  'trailing': ON [0, d*Ts), the
%                 sample sees the iL valley (bias -dI/2 = -0.6 A at the
%                 nominal point) and the end-of-OFF Vo.  'center': up-down
%                 carrier with its valley at the sample, ON [0, d*Ts/2) and
%                 [Ts - d*Ts/2, Ts), so the sample is the middle of the ON
%                 pulse = period-average iL in CCM.  Like delay = 0, both
%                 assume the new duty acts right at the sampling instant.
%                 Default = Pplant.pwm if that field exists, else 'trailing'.
%     Pplant      parameter struct for the PLANT only (default P).  Use it for
%                 L/C/ESR/DCR mismatch tests: controllers keep using P.
%     T_end       run length override; N = fix(T_end/Ts) as in Python
%     iL0, Vo0    initial state (default 0 A and P.Vin)
%
%   out fields (N x 1, sample k at t = (k-1)*Ts):
%     t, iL, Vo           true states (switched plant: Vo = terminal voltage
%                         incl. ESR at the sampling instant; Vc = capacitor)
%     d                   duty (gate for FCS-MPC) applied during period k
%     d_cmd               duty computed at sample k (differs from d with delay)
%     IL_ref, IL_ff       outer-loop outputs (NaN for single_pi)
%     Vo_meas, iL_meas    what the controller saw
%     R, Vin              load and input-voltage profiles
%     J (+ u = d)         FCS-MPC only
%     switched plant only, per period k (k = N is NaN): Vc, Vo_min, Vo_max,
%     iL_min, iL_max, iL_avg, Vo_avg, dcm  (iL_avg(k) - iL(k) = sampling bias)
%     info                resolved options
%   The controller is evaluated at every sample including the last one; the
%   state update stops at sample N, so samples 1..N match the notebook arrays
%   (the notebook leaves u(N) = 0 and does not compute it).

if nargin < 3 || isempty(P)
    P = boost_params();
end
if nargin < 4 || isempty(opts)
    opts = struct();
end

name = lower(strrep(ctrl, '-', '_'));
switch name
    case {'fcsmpc', 'fcs_mpc', 'mpc'}
        cid = 1; name = 'fcsmpc';
    case {'cascade_pi', 'pi'}
        cid = 2; name = 'cascade_pi';
    case {'cascade_pid', 'pid'}
        cid = 3; name = 'cascade_pid';
    case 'smc'
        cid = 4;
    case {'single_pi', 'spi'}
        cid = 5; name = 'single_pi';
    otherwise
        error('sim_boost:ctrl', 'unknown controller ''%s''', ctrl);
end
if nargin < 2 || isempty(gains)
    gains = P.gains_baseline.(name);
end
gains = double(gains(:)');
nreq = [1 2 4 2 2];
if numel(gains) ~= nreq(cid)
    error('sim_boost:gains', '%s needs %d gains, got %d', name, nreq(cid), numel(gains));
end

Ts = P.Ts;
Vref = P.Vref;
D_ss = P.D_ss;
DMIN = P.DMIN;
DMAX = P.DMAX;
KPV = getopt(opts, 'KPV', P.KPV);
KIV = getopt(opts, 'KIV', P.KIV);
IL_MAX = getopt(opts, 'IL_MAX', P.IL_MAX);
T_end = getopt(opts, 'T_end', P.T_end);
N = fix(T_end / Ts);
t = (0:N - 1)' * Ts;

lspec = getopt(opts, 'load', []);
if isempty(lspec)
    lspec = struct('kind', 'step', 'R0', P.R0, 'R_new', P.R1, 't_step', P.t_step);
end
if ~isfield(lspec, 'R0')
    lspec.R0 = P.R0;
end
Rk = load_profile(t, lspec);
vspec = getopt(opts, 'vin', []);
if isempty(vspec)
    vspec = struct('kind', 'const');
end
if ~isfield(vspec, 'Vin')
    vspec.Vin = P.Vin;
end
Vk = vin_profile(t, vspec);

ff_mode = lower(getopt(opts, 'ff_mode', 'oracle'));
switch ff_mode
    case 'oracle'
        fid = 1;
    case 'measured'
        fid = 2;
    case 'none'
        fid = 3;
    otherwise
        error('sim_boost:ff_mode', 'ff_mode must be oracle, measured or none');
end
delay = round(getopt(opts, 'delay', 0));
plant = lower(getopt(opts, 'plant', 'avg'));
switched = strcmp(plant, 'switched');
if ~switched && ~strcmp(plant, 'avg')
    error('sim_boost:plant', 'plant must be ''avg'' or ''switched''');
end
nsub = getopt(opts, 'nsub', 100);
Pp = getopt(opts, 'Pplant', P);
pwm = getopt(opts, 'pwm', getopt(Pp, 'pwm', 'trailing'));
if ~ischar(pwm)                    % numeric 0/1 accepted
    if pwm ~= 0
        pwm = 'center';
    else
        pwm = 'trailing';
    end
end
pwm = lower(pwm);
switch pwm
    case 'trailing'
    case {'center', 'centre'}
        pwm = 'center';
    otherwise
        error('sim_boost:pwm', 'pwm must be ''trailing'' or ''center''');
end
if switched
    Pp.pwm = pwm;                  % read by plant_switched_step
elseif strcmp(pwm, 'center')
    warning('sim_boost:pwm', 'opts.pwm only affects the switched plant (opts.plant = ''switched'')');
end
if ~switched && (getopt(Pp, 'rL', 0) ~= 0 || getopt(Pp, 'rC', 0) ~= 0 || ...
        getopt(Pp, 'Ron', 0) ~= 0 || getopt(Pp, 'VF', 0) ~= 0)
    warning('sim_boost:parasitics', ...
        'averaged plant ignores rL/rC/Ron/VF; use opts.plant = ''switched''');
end

% sensing: noise is pre-drawn (one randn per active channel per sample,
% column-major => same stream as drawing Vo, iL, io, Vin in turn each step)
sd = [getopt(opts, 'noise_std_v', 0), getopt(opts, 'noise_std_i', 0), ...
      getopt(opts, 'noise_std_io', 0), getopt(opts, 'noise_std_vin', 0)];
bits = getopt(opts, 'adc_bits', 0);
rV =getopt(opts, 'Vo_adc_range', P.Vo_adc_range);
rI = getopt(opts, 'iL_adc_range', P.iL_adc_range);
rIo = getopt(opts, 'io_adc_range', P.io_adc_range);
rVin = getopt(opts, 'vin_adc_range', P.vin_adc_range);
seed = getopt(opts, 'seed', 0);
noise = zeros(4, N);
act = find(sd > 0);
if ~isempty(act)
    rng_saved = rng;
    rng(seed, 'twister');
    z = randn(numel(act), N);
    rng(rng_saved);
    for j = 1:numel(act)
        noise(act(j), :) = sd(act(j)) * z(j, :);
    end
end
sense_on = (bits > 0) || ~isempty(act);

% states and logs
iL = zeros(N, 1);
Vo = zeros(N, 1);
iL(1) = getopt(opts, 'iL0', 0.0);
Vo(1) = getopt(opts, 'Vo0', P.Vin);
d = zeros(N, 1);
d_cmd = zeros(N, 1);
IL_ref_log = nan(N, 1);
IL_ff_log = nan(N, 1);
Vo_meas = zeros(N, 1);
iL_meas = zeros(N, 1);
J_log = [];
if cid == 1
    J_log = zeros(N, 1);
end
if switched
    Vc = zeros(N, 1);
    Vc(1) = Vo(1);
    Vo_min = nan(N, 1); Vo_max = nan(N, 1);
    iL_min = nan(N, 1); iL_max = nan(N, 1);
    iL_avg = nan(N, 1); Vo_avg = nan(N, 1);
    dcm = false(N, 1);
end

int_v = 0.0; int_i = 0.0; ei_prev = 0.0; d_filt = 0.0; u_prev = 0;
if cid == 1
    d_init = 0;
else
    d_init = D_ss;
end
dbuf = d_init * ones(1, delay);
g1 = gains(1);
if numel(gains) >= 2
    g2 = gains(2);
end
if cid == 3
    g3 = gains(3);
    g4 = gains(4);
end
L_model = P.L;
Vin_nom = P.Vin;
IL_ref = NaN;
IL_ff = NaN;

for k = 1:N
    R = Rk(k);
    Vin_k = Vk(k);
    Vo_k = Vo(k);
    iL_k = iL(k);
    if sense_on
        Vo_m = sense_adc(Vo_k, noise(1, k), bits, rV(1), rV(2));
        iL_m = sense_adc(iL_k, noise(2, k), bits, rI(1), rI(2));
    else
        Vo_m = Vo_k;
        iL_m = iL_k;
    end

    if cid ~= 5
        if fid == 1
            IL_ff = (Vref * Vref) / (Vin_k * R);
            Vin_model = Vin_k;
        elseif fid == 2
            io_m = sense_adc(Vo_k / R, noise(3, k), bits, rIo(1), rIo(2));
            vin_m = sense_adc(Vin_k, noise(4, k), bits, rVin(1), rVin(2));
            IL_ff = Vref * io_m / max(vin_m, 1e-3);
            Vin_model = vin_m;
        else
            IL_ff = 0.0;
            Vin_model = Vin_nom;
        end
        [IL_ref, int_v] = ctrl_outer_pi(Vo_m, int_v, IL_ff, Vref, KPV, KIV, IL_MAX, Ts);
    end

    switch cid
        case 1
            [dc, Jk] = ctrl_fcsmpc(iL_m, Vo_m, IL_ref, u_prev, g1, Vin_model, L_model, Ts);
            u_prev = dc;
            J_log(k) = Jk;
        case 2
            [dc, int_i] = ctrl_inner_pi(IL_ref - iL_m, int_i, g1, g2, D_ss, DMIN, DMAX, Ts);
        case 3
            [dc, int_i, ei_prev, d_filt] = ctrl_inner_pid(IL_ref - iL_m, int_i, ei_prev, d_filt, ...
                g1, g2, g3, g4, D_ss, DMIN, DMAX, Ts);
        case 4
            dc = ctrl_smc(IL_ref - iL_m, g1, g2, D_ss, DMIN, DMAX);
        otherwise
            [dc, int_v] = ctrl_single_pi(Vref - Vo_m, int_v, g1, g2, D_ss, DMIN, DMAX, Ts);
    end

    if delay == 0
        da = dc;
    else
        da = dbuf(1);
        dbuf = [dbuf(2:end), dc];
    end
    d(k) = da;
    d_cmd(k) = dc;
    IL_ref_log(k) = IL_ref;
    IL_ff_log(k) = IL_ff;
    Vo_meas(k) = Vo_m;
    iL_meas(k) = iL_m;

    if k < N
        if switched
            [iL(k + 1), Vc(k + 1), inf_k] = plant_switched_step(iL_k, Vc(k), da, R, Vin_k, Pp, nsub);
            Vo(k + 1) = inf_k.Vout_end;
            Vo_min(k) = inf_k.Vout_min;
            Vo_max(k) = inf_k.Vout_max;
            iL_min(k) = inf_k.iL_min;
            iL_max(k) = inf_k.iL_max;
            iL_avg(k) = inf_k.iL_avg;
            Vo_avg(k) = inf_k.Vout_avg;
            dcm(k) = inf_k.dcm;
        else
            [iL(k + 1), Vo(k + 1)] = plant_avg_step(iL_k, Vo_k, da, R, Vin_k, Pp.L, Pp.C, Ts);
        end
    end
end

out.t = t;
out.iL = iL;
out.Vo = Vo;
out.d = d;
out.d_cmd = d_cmd;
out.IL_ref = IL_ref_log;
out.IL_ff = IL_ff_log;
out.Vo_meas = Vo_meas;
out.iL_meas = iL_meas;
out.R = Rk;
out.Vin = Vk;
out.J = J_log;
if cid == 1
    out.u = d;
end
if switched
    out.Vc = Vc;
    out.Vo_min = Vo_min;
    out.Vo_max = Vo_max;
    out.iL_min = iL_min;
    out.iL_max = iL_max;
    out.iL_avg = iL_avg;
    out.Vo_avg = Vo_avg;
    out.dcm = dcm;
end
out.info = struct('ctrl', name, 'gains', gains, 'ff_mode', ff_mode, 'delay', delay, ...
    'plant', plant, 'nsub', nsub, 'pwm', pwm, 'KPV', KPV, 'KIV', KIV, 'IL_MAX', IL_MAX, ...
    'noise_std', sd, 'adc_bits', bits, 'seed', seed, 'N', N, 'Ts', Ts);
out.info.load = lspec;
out.info.vin = vspec;
end

function v = getopt(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
