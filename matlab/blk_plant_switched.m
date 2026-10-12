function [xp_next, env] = blk_plant_switched(xp, d, R, Vin, PP)
%BLK_PLANT_SWITCHED  Cycle-resolved (switched) boost plant for a MATLAB Function block.
%   [xp_next, env] = blk_plant_switched(xp, d, R, Vin, PP)
%     xp  = [iL; vC; Vout] (Unit Delay 'xp', initial condition [iL0; Vo0; Vo0])
%           iL, vC: inductor current / capacitor voltage at the period start;
%           Vout: terminal voltage at the end of the previous period (incl.
%           ESR), which is what the controller measures (= sim_boost Vo)
%     d   duty of the period starting now (FCS-MPC: gate 0/1)
%     R   load [ohm], Vin input voltage [V]
%     PP  1x10 [L C Ts rL rC Ron VF nsub sync pwm_center] (make_plant_params)
%   xp_next = [iL1; vC1; Vout_end] after one period Ts.
%   env = [Vo_min; Vo_max; iL_min; iL_max; iL_avg; Vo_avg; dcm] over the
%   period that starts at this sample (dcm 0/1).
%   Calls sw_boost_period (exact matrix-exponential solution of each
%   topology).  sim_boost's switched branch uses the vectorised twin
%   plant_switched_step, which agrees to rounding (~1e-12), not bit-exactly.
%#codegen
xp_next = zeros(3, 1);
env = zeros(7, 1);
[iL1, vC1, Vout_end, iL_min, iL_max, Vout_min, Vout_max, iL_avg, Vout_avg, dcm] = ...
    sw_boost_period(xp(1), xp(2), d, R, Vin, PP(1), PP(2), PP(3), PP(4), PP(5), ...
    PP(6), PP(7), PP(8), PP(9), PP(10));
xp_next(1) = iL1;
xp_next(2) = vC1;
xp_next(3) = Vout_end;
env(1) = Vout_min;
env(2) = Vout_max;
env(3) = iL_min;
env(4) = iL_max;
env(5) = iL_avg;
env(6) = Vout_avg;
env(7) = double(dcm);
end
