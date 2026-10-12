function xp_next = blk_plant_avg(xp, d, R, Vin, PP)
%BLK_PLANT_AVG  Averaged boost plant (forward Euler) for a MATLAB Function block.
%   xp_next = blk_plant_avg(xp, d, R, Vin, PP)
%     xp  = [iL; Vo] state (Unit Delay 'xp', initial condition [iL0; Vo0])
%     d   applied duty (FCS-MPC: gate 0/1), R load [ohm], Vin input [V]
%     PP  1x10 plant vector from make_plant_params; only PP(1) = L,
%         PP(2) = C and PP(3) = Ts are used here
%   Calls plant_avg_step, i.e. identical to the sim_boost 'avg' plant.
%#codegen
xp_next = zeros(2, 1);
[iL1, Vo1] = plant_avg_step(xp(1), xp(2), d, R, Vin, PP(1), PP(2), PP(3));
xp_next(1) = iL1;
xp_next(2) = Vo1;
end
