function [d, int_v] = ctrl_single_pi(ev, int_v, Kp_v, Ki_v, D_ss, DMIN, DMAX, Ts)
%CTRL_SINGLE_PI  Single-loop voltage-mode PI acting directly on the duty.
%   [d, int_v] = ctrl_single_pi(ev, int_v, Kp_v, Ki_v, D_ss, DMIN, DMAX, Ts)
%     ev = Vref - Vo_meas ; d_raw = D_ss + Kp_v*ev + Ki_v*int_v ; d = clip(d_raw)
%     int_v = int_v + (ev*Ts + (d - d_raw)/(Ki_v + 1e-12))
%   Legacy note: a plain PI on Vo of a boost converter (right-half-plane
%   zero, no current loop) is unstable with the notebook gains; it is kept
%   only to reproduce the legacy baseline.
%#codegen
d_raw = D_ss + Kp_v * ev + Ki_v * int_v;
if d_raw > DMAX
    d = DMAX;
elseif d_raw < DMIN
    d = DMIN;
else
    d = d_raw;
end
int_v = int_v + (ev * Ts + (d - d_raw) / (Ki_v + 1e-12));
end
