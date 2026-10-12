function [d, IL_ref, xc_next, meas] = blk_controller(truth, noise, xc, CP)
%BLK_CONTROLLER  All five boost controllers in one MATLAB Function block.
%   [d, IL_ref, xc_next, meas] = blk_controller(truth, noise, xc, CP)
%     truth = [Vo; iL; io; Vin; R]  true plant signals at this sample
%             (io = Vo/R from a Divide block)
%     noise = [nVo; niL; nio; nVin] sensor noise samples (root Inport,
%             zeros when no external input is given)
%     xc    = [int_v; int_i; ei_prev; d_filt; u_prev; dbuf1; dbuf2; dbuf3]
%             (Unit Delay 'xc', initial condition X0 from make_ctrl_params)
%             u_prev holds the last commanded duty/gate (read by FCS-MPC
%             only); dbuf is the computation-delay FIFO, oldest first
%     CP    1x32 parameter vector from make_ctrl_params:
%             1 ctrl id (1 fcsmpc, 2 cascade_pi, 3 cascade_pid, 4 smc,
%             5 single_pi)   2:5 gains g1..g4   6 ff_mode (1 oracle,
%             2 measured, 3 none)   7 delay 0..3   8 adc_bits (0 = off)
%             9 KPV  10 KIV  11 IL_MAX  12 Vref  13 D_ss  14 DMIN  15 DMAX
%             16 L_model  17 Ts  18:25 ADC ranges [Vo lo hi, iL lo hi,
%             io lo hi, Vin lo hi]  26 Vin_nom  27:32 reserved
%   Outputs: d applied duty (gate for FCS-MPC), IL_ref outer-loop current
%   reference (NaN for single_pi), xc_next, meas = [Vo_m; iL_m; io_m; vin_m]
%   (0 for channels that are not sensed in this mode).
%   Per-sample logic and floating-point order are those of sim_boost, so a
%   model built around this block reproduces sim_boost bit-exactly
%   (checked by emulate_simulink / tests/test_simulink_blocks).
%#codegen
cid = CP(1);
g1 = CP(2); g2 = CP(3); g3 = CP(4); g4 = CP(5);
fid = CP(6);
delay = CP(7);
bits = CP(8);
KPV = CP(9); KIV = CP(10); IL_MAX = CP(11);
Vref = CP(12); D_ss = CP(13); DMIN = CP(14); DMAX = CP(15);
L_model = CP(16); Ts = CP(17);
Vin_nom = CP(26);

Vo_k = truth(1);
iL_k = truth(2);
io_k = truth(3);
Vin_k = truth(4);
R = truth(5);

int_v = xc(1); int_i = xc(2); ei_prev = xc(3); d_filt = xc(4); u_prev = xc(5);
b1 = xc(6); b2 = xc(7); b3 = xc(8);

meas = zeros(4, 1);
% bits = 0 and zero noise give x + 0 = x, i.e. the unsensed sim_boost path
Vo_m = sense_adc(Vo_k, noise(1), bits, CP(18), CP(19));
iL_m = sense_adc(iL_k, noise(2), bits, CP(20), CP(21));
meas(1) = Vo_m;
meas(2) = iL_m;

IL_ref = NaN;
Vin_model = Vin_nom;
if cid ~= 5
    if fid == 1
        IL_ff = (Vref * Vref) / (Vin_k * R);
        Vin_model = Vin_k;
    elseif fid == 2
        io_m = sense_adc(io_k, noise(3), bits, CP(22), CP(23));
        vin_m = sense_adc(Vin_k, noise(4), bits, CP(24), CP(25));
        IL_ff = Vref * io_m / max(vin_m, 1e-3);
        Vin_model = vin_m;
        meas(3) = io_m;
        meas(4) = vin_m;
    else
        IL_ff = 0.0;
        Vin_model = Vin_nom;
    end
    [IL_ref, int_v] = ctrl_outer_pi(Vo_m, int_v, IL_ff, Vref, KPV, KIV, IL_MAX, Ts);
end

if cid == 1
    [dc, ~] = ctrl_fcsmpc(iL_m, Vo_m, IL_ref, u_prev, g1, Vin_model, L_model, Ts);
elseif cid == 2
    [dc, int_i] = ctrl_inner_pi(IL_ref - iL_m, int_i, g1, g2, D_ss, DMIN, DMAX, Ts);
elseif cid == 3
    [dc, int_i, ei_prev, d_filt] = ctrl_inner_pid(IL_ref - iL_m, int_i, ei_prev, d_filt, ...
        g1, g2, g3, g4, D_ss, DMIN, DMAX, Ts);
elseif cid == 4
    dc = ctrl_smc(IL_ref - iL_m, g1, g2, D_ss, DMIN, DMAX);
else
    [dc, int_v] = ctrl_single_pi(Vref - Vo_m, int_v, g1, g2, D_ss, DMIN, DMAX, Ts);
end

% computation delay: the duty computed now is applied 'delay' samples later
if delay == 0
    d = dc;
elseif delay == 1
    d = b1;
    b1 = dc;
elseif delay == 2
    d = b1;
    b1 = b2;
    b2 = dc;
else
    d = b1;
    b1 = b2;
    b2 = b3;
    b3 = dc;
end

xc_next = zeros(8, 1);
xc_next(1) = int_v;
xc_next(2) = int_i;
xc_next(3) = ei_prev;
xc_next(4) = d_filt;
xc_next(5) = dc;
xc_next(6) = b1;
xc_next(7) = b2;
xc_next(8) = b3;
end
