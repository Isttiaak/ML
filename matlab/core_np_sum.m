function s = core_np_sum(x)
%CORE_NP_SUM  Sum with numpy's pairwise-summation order (np.sum / np.mean).
%   Reproduces numpy's float64 add.reduce bit-for-bit: blocks of <= 128
%   elements use 8 interleaved accumulators, larger blocks are split in two
%   halves (first half size rounded down to a multiple of 8).
x = x(:);
s = 0 + pw(x, 1, numel(x));
end

function r = pw(x, i0, n)
if n < 8
    r = -0.0;
    for i = i0:(i0 + n - 1)
        r = r + x(i);
    end
elseif n <= 128
    acc = x(i0:i0 + 7);
    m = n - mod(n, 8);
    i = 8;
    while i < m
        acc = acc + x(i0 + i:i0 + i + 7);
        i = i + 8;
    end
    r = ((acc(1) + acc(2)) + (acc(3) + acc(4))) + ((acc(5) + acc(6)) + (acc(7) + acc(8)));
    for j = i:(n - 1)
        r = r + x(i0 + j);
    end
else
    n2 = fix(n / 2);
    n2 = n2 - mod(n2, 8);
    r = pw(x, i0, n2) + pw(x, i0 + n2, n - n2);
end
end
