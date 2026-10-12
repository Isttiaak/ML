function [CP, X0, name] = make_ctrl_params(ctrl, gains, P, opts)
%MAKE_CTRL_PARAMS  Parameter vector CP and initial state X0 for blk_controller.
%   [CP, X0] = make_ctrl_params(ctrl, gains, P, opts)
%   [CP, X0, name] = ...   also returns the canonical controller name
%   Same arguments, aliases, defaults and errors as sim_boost:
%     ctrl  'fcsmpc' | 'cascade_pi' | 'cascade_pid' | 'smc' | 'single_pi'
%           (aliases fcs_mpc mpc pi pid spi, '-' accepted for '_')
%     gains notebook order ([] = P.gains_baseline.<ctrl>)
%     P     boost_params() struct ([] = default)
%     opts  fields used: ff_mode ('oracle'|'measured'|'none'), delay (0..3),
%           adc_bits, Vo_adc_range, iL_adc_range, io_adc_range,
%           vin_adc_range, KPV, KIV, IL_MAX.  Other fields are ignored.
%   CP (1x32 row, block Parameter 'CP'):
%     1 ctrl id (1 fcsmpc, 2 cascade_pi, 3 cascade_pid, 4 smc, 5 single_pi)
%     2:5 gains g1..g4 (unused = 0)  6 ff_mode (1 oracle, 2 measured, 3 none)
%     7 delay  8 adc_bits  9 KPV  10 KIV  11 IL_MAX  12 Vref  13 D_ss
%     14 DMIN  15 DMAX  16 L_model (= P.L)  17 Ts  18:25 ADC ranges
%     [Vo lo hi, iL lo hi, io lo hi, vin lo hi]  26 Vin_nom (= P.Vin)
%     27:32 reserved (0)
%   X0 (8x1, initial condition of Unit Delay 'xc'):
%     [int_v int_i ei_prev d_filt u_prev dbuf1 dbuf2 dbuf3]' =
%     [0 0 0 0 0 d_init d_init d_init]', d_init = D_ss (0 for fcsmpc).
%   The delay FIFO of the block has 3 slots, so delay > 3 is an error.

if nargin < 3 || isempty(P)
    P = boost_params();
end
if nargin < 4 || isempty(opts)
    opts = struct();
end
if ~ischar(ctrl)
    error('make_ctrl_params:ctrl', 'ctrl must be a char array');
end
name = lower(strrep(ctrl, '-', '_'));
switch name
    case {'fcsmpc', 'fcs_mpc', 'mpc'}
        cid = 1; name = 'fcsmpc';
    case {'cascade_pi', 'pi'}
        cid = 2; name = 'cascade_pi';
    case {'cascade_pid', 'pid'}
        cid = 3; name = 'cascade_pid';
    case 'smc'
        cid = 4;
    case {'single_pi', 'spi'}
        cid = 5; name = 'single_pi';
    otherwise
        error('make_ctrl_params:ctrl', 'unknown controller ''%s''', ctrl);
end
if nargin < 2 || isempty(gains)
    gains = P.gains_baseline.(name);
end
gains = double(gains(:)');
nreq = [1 2 4 2 2];
if numel(gains) ~= nreq(cid)
    error('make_ctrl_params:gains', '%s needs %d gains, got %d', name, nreq(cid), numel(gains));
end

ff_mode = lower(getopt(opts, 'ff_mode', 'oracle'));
switch ff_mode
    case 'oracle'
        fid = 1;
    case 'measured'
        fid = 2;
    case 'none'
        fid = 3;
    otherwise
        error('make_ctrl_params:ff_mode', 'ff_mode must be oracle, measured or none');
end
delay = round(getopt(opts, 'delay', 0));
if ~isscalar(delay) || delay < 0 || delay > 3
    error('make_ctrl_params:delay', ...
        'delay must be 0..3 samples (blk_controller has a 3-slot FIFO), got %g', delay);
end
bits = getopt(opts, 'adc_bits', 0);
rV = getopt(opts, 'Vo_adc_range', P.Vo_adc_range);
rI = getopt(opts, 'iL_adc_range', P.iL_adc_range);
rIo = getopt(opts, 'io_adc_range', P.io_adc_range);
rVin = getopt(opts, 'vin_adc_range', P.vin_adc_range);
rng4 = [rV(:)' rI(:)' rIo(:)' rVin(:)'];
if numel(rng4) ~= 8
    error('make_ctrl_params:range', 'ADC ranges must be [lo hi] pairs');
end

g = zeros(1, 4);
g(1:numel(gains)) = gains;
CP = zeros(1, 32);
CP(1) = cid;
CP(2:5) = g;
CP(6) = fid;
CP(7) = delay;
CP(8) = bits;
CP(9) = getopt(opts, 'KPV', P.KPV);
CP(10) = getopt(opts, 'KIV', P.KIV);
CP(11) = getopt(opts, 'IL_MAX', P.IL_MAX);
CP(12) = P.Vref;
CP(13) = P.D_ss;
CP(14) = P.DMIN;
CP(15) = P.DMAX;
CP(16) = P.L;
CP(17) = P.Ts;
CP(18:25) = rng4;
CP(26) = P.Vin;

if cid == 1
    d_init = 0;
else
    d_init = P.D_ss;
end
X0 = [0; 0; 0; 0; 0; d_init; d_init; d_init];
end

function v = getopt(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
