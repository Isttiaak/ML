function out = emulate_simulink(ctrl, gains, P, opts)
%EMULATE_SIMULINK  Octave/MATLAB emulation of the Simulink models boost_avg / boost_cycle.
%   out = emulate_simulink(ctrl, gains, P, opts)
%   Same arguments and output fields as sim_boost (opts.plant 'avg' ->
%   model boost_avg, 'switched' -> boost_cycle; opts.Pplant, opts.pwm,
%   opts.nsub supported), but every sample is computed ONLY by the MATLAB
%   Function block functions, called in the order a fixed-step discrete
%   Simulink model executes them:
%     1. Unit Delay outputs        k, xp, xc (states of the previous step)
%     2. Scenario block            [R, Vin, k_next] = blk_scenario(k, LP, VP, Ts)
%     3. Demux / Divide / Mux      iL = xp(1), Vo = xp(2) (avg) or xp(3)
%                                  (switched), io = Vo / R,
%                                  truth = [Vo; iL; io; Vin; R]
%     4. Root Inport 'noise'       noise(:, k) (zeros when there is no noise)
%     5. Controller block          [d, IL_ref, xc_next, meas] =
%                                      blk_controller(truth, noise, xc, CP)
%     6. Plant block               xp_next = blk_plant_avg(xp, d, R, Vin, PP)
%                                  [xp_next, env] = blk_plant_switched(...)
%     7. Unit Delay updates        k = k_next, xp = xp_next, xc = xc_next
%   All parameters and initial states come from emu_model_vars (CP, X0, LP,
%   VP, PP, XP0, noise), which a Simulink run script can use as well.
%
%   Logged like sim_boost (sample k at t = (k-1)*Ts): t iL Vo d d_cmd IL_ref
%   Vo_meas iL_meas R Vin, plus io_meas, vin_meas (= meas(3:4)).
%   d_cmd = xc_next(5).  IL_ff, J (FCS-MPC cost) and u are derived after
%   the run from logged block signals with the formulas of sim_boost; they
%   are not block outputs.  Switched model: Vc = xp(2) and the env fields
%   Vo_min Vo_max iL_min iL_max iL_avg Vo_avg dcm.  The plant block also
%   runs at the last sample; its env there is set to NaN/false like in
%   sim_boost, which does not step the plant after sample N.
%   out.info = sim_boost's info fields + model, CP, X0, LP, VP, PP, XP0.
%   With the averaged plant the result equals sim_boost bit-exactly
%   (tests/test_simulink_blocks).  boost_cycle uses sw_boost_period while
%   sim_boost uses plant_switched_step: equal to rounding (~1e-12).
%   Restrictions of the block version: delay <= 3, Pplant.Ts == P.Ts.

if nargin < 2
    gains = [];
end
if nargin < 3 || isempty(P)
    P = boost_params();
end
if nargin < 4 || isempty(opts)
    opts = struct();
end

V = emu_model_vars(ctrl, gains, P, opts);
CP = V.CP; LP = V.LP; VP = V.VP; PP = V.PP; Ts = V.Ts; N = V.N;
noise = V.noise;
switched = strcmp(V.model, 'boost_cycle');

% logs
L_iL = zeros(N, 1); L_Vo = zeros(N, 1); L_d = zeros(N, 1); L_dc = zeros(N, 1);
L_ref = zeros(N, 1); L_meas = zeros(N, 4); L_R = zeros(N, 1); L_Vin = zeros(N, 1);
L_up = zeros(N, 1);
if switched
    L_Vc = zeros(N, 1);
    L_env = zeros(N, 7);
end

% Unit Delay states
k_st = 0;
xp = V.XP0;
xc = V.X0;
for k = 1:N
    [R, Vin, k_next] = blk_scenario(k_st, LP, VP, Ts);
    iL = xp(1);
    if switched
        Vo = xp(3);
    else
        Vo = xp(2);
    end
    io = Vo / R;
    truth = [Vo; iL; io; Vin; R];
    [d, IL_ref, xc_next, meas] = blk_controller(truth, noise(:, k), xc, CP);
    if switched
        [xp_next, env] = blk_plant_switched(xp, d, R, Vin, PP);
        L_Vc(k) = xp(2);
        L_env(k, :) = env.';
    else
        xp_next = blk_plant_avg(xp, d, R, Vin, PP);
    end
    L_iL(k) = iL; L_Vo(k) = Vo; L_d(k) = d; L_dc(k) = xc_next(5);
    L_ref(k) = IL_ref; L_meas(k, :) = meas.'; L_R(k) = R; L_Vin(k) = Vin;
    L_up(k) = xc(5);
    k_st = k_next;
    xp = xp_next;
    xc = xc_next;
end

% ---- derived logs (post-processing of logged signals, not model blocks)
cid = CP(1);
Vref = CP(12);
fid = CP(6);
IL_ff = nan(N, 1);
if cid ~= 5
    if fid == 1
        IL_ff = (Vref * Vref) ./ (L_Vin .* L_R);
    elseif fid == 2
        IL_ff = Vref * L_meas(:, 3) ./ max(L_meas(:, 4), 1e-3);
    else
        IL_ff = zeros(N, 1);
    end
end
J = [];
if cid == 1
    J = zeros(N, 1);
    for k = 1:N
        if fid == 1
            Vin_model = L_Vin(k);
        elseif fid == 2
            Vin_model = L_meas(k, 4);
        else
            Vin_model = CP(26);
        end
        [~, J(k)] = ctrl_fcsmpc(L_meas(k, 2), L_meas(k, 1), L_ref(k), L_up(k), CP(2), ...
            Vin_model, CP(16), Ts);
    end
end

out.t = V.t;
out.iL = L_iL;
out.Vo = L_Vo;
out.d = L_d;
out.d_cmd = L_dc;
out.IL_ref = L_ref;
out.IL_ff = IL_ff;
out.Vo_meas = L_meas(:, 1);
out.iL_meas = L_meas(:, 2);
out.io_meas = L_meas(:, 3);
out.vin_meas = L_meas(:, 4);
out.R = L_R;
out.Vin = L_Vin;
out.J = J;
if cid == 1
    out.u = L_d;
end
if switched
    L_env(N, :) = NaN;
    out.Vc = L_Vc;
    out.Vo_min = L_env(:, 1);
    out.Vo_max = L_env(:, 2);
    out.iL_min = L_env(:, 3);
    out.iL_max = L_env(:, 4);
    out.iL_avg = L_env(:, 5);
    out.Vo_avg = L_env(:, 6);
    out.dcm = L_env(:, 7) == 1;
end
nreq = [1 2 4 2 2];
out.info = struct('ctrl', V.name, 'gains', CP(2:1 + nreq(cid)), ...
    'ff_mode', lower(getopt(opts, 'ff_mode', 'oracle')), 'delay', CP(7), ...
    'plant', V.plant, 'nsub', PP(8), 'pwm', V.pwm, 'KPV', CP(9), 'KIV', CP(10), 'IL_MAX', CP(11), ...
    'noise_std', V.sd, 'adc_bits', CP(8), 'seed', V.seed, 'N', N, 'Ts', Ts);
out.info.load = V.lspec;
out.info.vin = V.vspec;
out.info.model = V.model;
out.info.CP = CP;
out.info.X0 = V.X0;
out.info.LP = LP;
out.info.VP = VP;
out.info.PP = PP;
out.info.XP0 = V.XP0;
end

function v = getopt(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
