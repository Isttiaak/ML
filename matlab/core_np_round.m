function y = core_np_round(x, n)
%CORE_NP_ROUND  numpy rounding round(np.float64, n) = rint(x*10^n)/10^n.
%   Differs from Python's round() (py_round) because x*10^n is rounded
%   first, e.g. core_np_round(70.40015,4) = 70.4002, py_round = 70.4001.
f = 10^n;
y = round_half_even(x * f) / f;
end
