function n = blk_noise_lookup(k, NZ)
%BLK_NOISE_LOOKUP  Optional exact noise source: column k+1 of a stored noise matrix.
%   n = blk_noise_lookup(k, NZ)
%     k   sample counter 0, 1, ... (the output of Unit Delay 'k')
%     NZ  4 x N noise matrix (block Parameter, e.g. emu_model_vars(...).noise)
%   n = NZ(:, k+1) as a 4x1 column, zeros outside 1..N.  Alternative to the
%   root Inport 'noise' that does not depend on how Simulink time-stamps
%   external input data (bit-exact by construction); NZ keeps its size when
%   only the seed changes, so it stays tunable under Fast Restart.
%#codegen
n = zeros(4, 1);
j = k + 1;
if j >= 1 && j <= size(NZ, 2)
    for r = 1:4
        n(r) = NZ(r, j);
    end
end
end
