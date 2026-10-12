function y = sense_adc(x, noise_sample, bits, lo, hi)
%SENSE_ADC  Sensor model: additive noise + optional uniform ADC quantisation.
%   y = sense_adc(x, noise_sample, bits, lo, hi)
%     noise_sample : pre-drawn noise value (e.g. std*randn), 0 for none;
%                    drawing it outside keeps this function deterministic.
%     bits <= 0    : no quantisation (y = x + noise_sample)
%     otherwise    : q = clip(rint((y-lo)/(hi-lo)*(2^bits-1)), 0, 2^bits-1)
%                    y = lo + q/(2^bits-1)*(hi-lo)
%   Matches sense() of the notebook (cell 12), including numpy's
%   round-half-even.  Values outside [lo, hi] saturate at the range ends.
%#codegen
y = x + noise_sample;
if bits > 0
    levels = 2^bits;
    q = round_half_even((y - lo) / (hi - lo) * (levels - 1));
    if q < 0
        q = 0;
    elseif q > levels - 1
        q = levels - 1;
    end
    y = lo + q / (levels - 1) * (hi - lo);
end
end
