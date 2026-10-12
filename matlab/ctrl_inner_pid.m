function [d, int_i, ei_prev, d_filt] = ctrl_inner_pid(ei, int_i, ei_prev, d_filt, Kp_i, Ki_i, Kd_i, Tf, D_ss, DMIN, DMAX, Ts)
%CTRL_INNER_PID  Inner current PID with filtered derivative, clamp, anti-windup.
%   [d, int_i, ei_prev, d_filt] = ctrl_inner_pid(ei, int_i, ei_prev, d_filt,
%                                   Kp_i, Ki_i, Kd_i, Tf, D_ss, DMIN, DMAX, Ts)
%     alpha  = Tf/(Tf + Ts)
%     d_filt = alpha*d_filt + (1 - alpha)*(ei - ei_prev)/Ts
%     d_raw  = D_ss + Kp_i*ei + Ki_i*int_i + Kd_i*d_filt ; d = clip(d_raw)
%     int_i  = int_i + (ei*Ts + (d - d_raw)/(Ki_i + 1e-12)) ; ei_prev = ei
%   States int_i, ei_prev, d_filt start at 0 (as in the notebook).
%#codegen
alpha = Tf / (Tf + Ts);
d_filt = alpha * d_filt + (1 - alpha) * (ei - ei_prev) / Ts;
d_raw = D_ss + Kp_i * ei + Ki_i * int_i + Kd_i * d_filt;
if d_raw > DMAX
    d = DMAX;
elseif d_raw < DMIN
    d = DMIN;
else
    d = d_raw;
end
int_i = int_i + (ei * Ts + (d - d_raw) / (Ki_i + 1e-12));
ei_prev = ei;
end
