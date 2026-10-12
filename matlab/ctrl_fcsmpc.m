function [u, J] = ctrl_fcsmpc(iL_meas, Vo_meas, IL_ref, u_prev, lsw, Vin_model, L, Ts)
%CTRL_FCSMPC  Finite-control-set MPC, one-step inductor-current prediction.
%   [u, J] = ctrl_fcsmpc(iL_meas, Vo_meas, IL_ref, u_prev, lsw, Vin_model, L, Ts)
%   For uc in {0,1}:
%     iL_p = iL_meas + Ts*(Vin_model - (1 - uc)*Vo_meas)/L      (Euler step)
%     Jc   = (iL_p - IL_ref)^2 + lsw*(uc - u_prev)^2
%   u = argmin Jc, scanning uc = 0 then 1 with strict '<' (tie keeps 0),
%   J = min cost.  u is held for the whole period Ts (gate signal, d = u).
%   Legacy caveat: with Ts = 50 us the current moves by about
%   Ts*Vin/L = 2.4 A per decision, so the current ripple is set by Ts, not
%   by a modulator.
%#codegen
J = Inf;
u = 0;
for uc = 0:1
    diL = (Vin_model - (1 - uc) * Vo_meas) / L;
    iL_p = iL_meas + Ts * diL;
    e = iL_p - IL_ref;
    du = uc - u_prev;
    Jc = e * e + lsw * (du * du);
    if Jc < J
        J = Jc;
        u = uc;
    end
end
end
