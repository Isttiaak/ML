function [IL_ref, int_v] = ctrl_outer_pi(Vo_meas, int_v, IL_ff, Vref, KPV, KIV, IL_MAX, Ts)
%CTRL_OUTER_PI  Outer voltage PI with feed-forward and back-calculation anti-windup.
%   [IL_ref, int_v] = ctrl_outer_pi(Vo_meas, int_v, IL_ff, Vref, KPV, KIV, IL_MAX, Ts)
%     ev     = Vref - Vo_meas
%     IL_raw = IL_ff + KPV*ev + KIV*int_v
%     IL_ref = clip(IL_raw, 0, IL_MAX)
%     int_v  = int_v + ev*Ts + (IL_ref - IL_raw)/(KIV + 1e-12)
%   Same floating-point order as outer_pi_step() in the notebook (cell 1).
%   IL_ff is supplied by the caller (legacy oracle Vref^2/(Vin*R), measured
%   Vref*io/Vin, or 0).  Simulink: feed int_v back through a Unit Delay
%   (initial condition 0) and call once per Ts.
%#codegen
ev = Vref - Vo_meas;
IL_raw = IL_ff + KPV * ev + KIV * int_v;
if IL_raw > IL_MAX
    IL_ref = IL_MAX;
elseif IL_raw < 0
    IL_ref = 0;
else
    IL_ref = IL_raw;
end
int_v = int_v + ev * Ts + (IL_ref - IL_raw) / (KIV + 1e-12);
end
