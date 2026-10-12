function [d, int_i] = ctrl_inner_pi(ei, int_i, Kp_i, Ki_i, D_ss, DMIN, DMAX, Ts)
%CTRL_INNER_PI  Inner current PI (cascade) with duty clamp and anti-windup.
%   [d, int_i] = ctrl_inner_pi(ei, int_i, Kp_i, Ki_i, D_ss, DMIN, DMAX, Ts)
%     ei    = IL_ref - iL_meas (computed by caller)
%     d_raw = D_ss + Kp_i*ei + Ki_i*int_i ;  d = clip(d_raw, DMIN, DMAX)
%     int_i = int_i + (ei*Ts + (d - d_raw)/(Ki_i + 1e-12))
%   The bracket in the last line reproduces Python's "int_i += a + b".
%#codegen
d_raw = D_ss + Kp_i * ei + Ki_i * int_i;
if d_raw > DMAX
    d = DMAX;
elseif d_raw < DMIN
    d = DMIN;
else
    d = d_raw;
end
int_i = int_i + (ei * Ts + (d - d_raw) / (Ki_i + 1e-12));
end
