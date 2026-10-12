function y = round_half_even(x)
%ROUND_HALF_EVEN  Round to nearest integer, ties to even (numpy.rint / np.round).
%   y = round_half_even(x) works element-wise.  MATLAB's round() sends ties
%   away from zero (2.5 -> 3); this returns 2 like numpy.
%#codegen
y = round(x);
tie = abs(x - fix(x)) == 0.5;          % exact: x - fix(x) has no rounding error
odd = mod(y, 2) ~= 0;
y = y - double(tie & odd) .* sign(x);
end
