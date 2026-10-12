function y = py_round(x, n)
%PY_ROUND  Python built-in round(float, n) for n >= 0.
%   Python rounds the exact binary value of x to n decimals (ties to even)
%   and returns the double nearest to that decimal.  sprintf('%.*f') does
%   the same correctly-rounded conversion, str2double converts back.
%   Element-wise; NaN/Inf pass through.  Not for code generation.
y = zeros(size(x));
for i = 1:numel(x)
    y(i) = str2double(sprintf('%.*f', n, x(i)));
end
end
