function d = ctrl_smc(sigma, K_s, phi, D_ss, DMIN, DMAX)
%CTRL_SMC  Boundary-layer sliding-mode current controller (static law).
%   d = ctrl_smc(sigma, K_s, phi, D_ss, DMIN, DMAX)
%     sigma = IL_ref - iL_meas ;  d = clip(D_ss + K_s*tanh(sigma/phi), DMIN, DMAX)
%   Note: numpy's SIMD tanh and the C library tanh can differ by 1 ulp, so
%   SMC trajectories match the Python reference to ~1e-13, not bit-exactly.
%#codegen
d_raw = D_ss + K_s * tanh(sigma / phi);
if d_raw > DMAX
    d = DMAX;
elseif d_raw < DMIN
    d = DMIN;
else
    d = d_raw;
end
end
