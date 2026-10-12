function S = drv_rob_scenarios(P, rob)
%DRV_ROB_SCENARIOS  Default robustness battery (used by run_robustness).
%   S = drv_rob_scenarios(P, rob)   rob = study_config().rob
%   Struct array with fields
%     name, label   identifier and plain-text table label
%     load, vin     load_profile / vin_profile specs
%     t_evt         event time for metrics_event [s]
%     pplant        struct of PLANT parameter overrides (absolute values,
%                   e.g. L = 0.8*P.L); controllers keep the nominal P
%     opts          extra sim_boost options for this scenario
%     mc            true: Monte Carlo over seeds 1..rob.mc_seeds
%     needs_switched  true: only meaningful on the switched plant
%   Scenarios (load step at 50 ms unless stated; every step is feasible,
%   R >= Vref^2/(Vin*IL_MAX) = 8 ohm at 24 V and 10 ohm at 19.2 V):
%     nominal       20 -> 10 ohm (-50 %)
%     load_m55      20 -> 8.9 ohm (-55.5 %, replaces the infeasible -75 %)
%     load_release  10 -> 20 ohm
%     load_ramp     20 -> 10 ohm linearly over 10 ms
%     vin_p20       Vin +20 % step at 60 ms, R = 20 ohm
%     vin_m20       Vin -20 % step at 60 ms, R = 20 ohm
%     L_m20, L_p20, C_m20, C_p20   plant L or C -20 % / +20 % (controllers
%                   and FCS-MPC model keep the nominal values), nominal step
%     parasitics    rL, rC, Ron, VF from rob.parasitics (switched plant)
%     delay_plus    computation delay rob.delay + rob.extra_delay samples
%     noise_mc      sensor noise on Vo, iL, io, Vin + ADC (rob.noise),
%                   nominal step, Monte Carlo
Vin = P.Vin;
stepL = struct('kind', 'step', 'R0', P.R0, 'R_new', P.R1, 't_step', P.t_step);
cV = struct('kind', 'const', 'Vin', Vin);
e = struct();
S = mk('nominal', 'Load step 20 to 10 Ohm (-50%)', stepL, cV, P.t_step, e, e, false, false);
S(end + 1) = mk('load_m55', 'Load step 20 to 8.9 Ohm (-55.5%)', ...
    struct('kind', 'step', 'R0', P.R0, 'R_new', 8.9, 't_step', P.t_step), cV, P.t_step, e, e, false, false);
S(end + 1) = mk('load_release', 'Load release 10 to 20 Ohm', ...
    struct('kind', 'step', 'R0', 10, 'R_new', 20, 't_step', P.t_step), cV, P.t_step, e, e, false, false);
S(end + 1) = mk('load_ramp', 'Load ramp 20 to 10 Ohm in 10 ms', ...
    struct('kind', 'ramp', 'R0', P.R0, 't0', P.t_step, 't1', P.t_step + 0.010, 'R_end', P.R1), ...
    cV, P.t_step, e, e, false, false);
S(end + 1) = mk('vin_p20', 'Vin step +20% at 60 ms (20 Ohm)', struct('kind', 'const', 'R0', P.R0), ...
    struct('kind', 'step', 'Vin', Vin, 'pct', 0.20, 't_step', 0.060), 0.060, e, e, false, false);
S(end + 1) = mk('vin_m20', 'Vin step -20% at 60 ms (20 Ohm)', struct('kind', 'const', 'R0', P.R0), ...
    struct('kind', 'step', 'Vin', Vin, 'pct', -0.20, 't_step', 0.060), 0.060, e, e, false, false);
S(end + 1) = mk('L_m20', 'Plant L -20%', stepL, cV, P.t_step, struct('L', 0.8 * P.L), e, false, false);
S(end + 1) = mk('L_p20', 'Plant L +20%', stepL, cV, P.t_step, struct('L', 1.2 * P.L), e, false, false);
S(end + 1) = mk('C_m20', 'Plant C -20%', stepL, cV, P.t_step, struct('C', 0.8 * P.C), e, false, false);
S(end + 1) = mk('C_p20', 'Plant C +20%', stepL, cV, P.t_step, struct('C', 1.2 * P.C), e, false, false);
pz = rob.parasitics;
S(end + 1) = mk('parasitics', sprintf('Parasitics rL %g, rC %g, Ron %g mOhm, VF %g V', ...
    1e3 * pz.rL, 1e3 * pz.rC, 1e3 * pz.Ron, pz.VF), stepL, cV, P.t_step, pz, e, false, true);
dtot = rob.delay + rob.extra_delay;
S(end + 1) = mk('delay_plus', sprintf('Computation delay %d samples', dtot), stepL, cV, P.t_step, e, ...
    struct('delay', dtot), false, false);
nz = rob.noise;
S(end + 1) = mk('noise_mc', sprintf('Noise %g mA / %g mV + %d-bit ADC (MC)', 1e3 * nz.noise_std_i, ...
    1e3 * nz.noise_std_v, nz.adc_bits), stepL, cV, P.t_step, e, nz, true, false);
end

function s = mk(name, label, lspec, vspec, t_evt, pplant, opts, mc, needs_sw)
s = struct('name', name, 'label', label, 'load', lspec, 'vin', vspec, 't_evt', t_evt, ...
    'pplant', pplant, 'opts', opts, 'mc', mc, 'needs_switched', needs_sw);
end
