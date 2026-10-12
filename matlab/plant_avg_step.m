function [iL1, Vo1] = plant_avg_step(iL, Vo, d, R, Vin_k, L, C, Ts)
%PLANT_AVG_STEP  One forward-Euler step of the ideal averaged boost model.
%   [iL1, Vo1] = plant_avg_step(iL, Vo, d, R, Vin_k, L, C, Ts)
%     diL = (Vin_k - (1 - d)*Vo)/L ;  dVo = ((1 - d)*iL - Vo/R)/C
%     iL1 = iL + Ts*diL ;  Vo1 = Vo + Ts*dVo   (both from the old state)
%   Identical to euler_step()/euler_step_gen() in the notebook.  It has no
%   switching ripple and no parasitics; use plant_switched_step for ripple
%   claims.  The inductor current is allowed to go negative (no DCM).
%#codegen
diL = (Vin_k - (1 - d) * Vo) / L;
dVo = ((1 - d) * iL - Vo / R) / C;
iL1 = iL + Ts * diL;
Vo1 = Vo + Ts * dVo;
end
