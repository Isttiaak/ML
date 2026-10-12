function [hdr, M] = core_read_csv(fname)
%CORE_READ_CSV  Read a numeric CSV with one header row.
%   [hdr, M] = core_read_csv(fname) -> hdr cellstr (1 x ncol), M (nrow x ncol).
%   Uses str2double, which converts the 17-significant-digit values written
%   by Python exactly (checked bit-for-bit in Octave 8.4).
txt = fileread(fname);
lines = regexp(txt, '\r?\n', 'split');
lines = lines(~cellfun(@isempty, lines));
hdr = strsplit(lines{1}, ',');
ncol = numel(hdr);
body = strjoin(lines(2:end), ',');
v = str2double(strsplit(body, ','));
M = reshape(v, ncol, []).';
end
