function s = emu_decode_cp(CP)
%EMU_DECODE_CP  Readable struct from a blk_controller parameter vector CP.
%   s = emu_decode_cp(CP)   inverse of make_ctrl_params (for checks and for
%   inspecting the 'CP' parameter of a Simulink model).  Fields: ctrl, cid,
%   gains (only the used ones), ff_mode, delay, adc_bits, KPV, KIV, IL_MAX,
%   Vref, D_ss, DMIN, DMAX, L_model, Ts, Vo_adc_range, iL_adc_range,
%   io_adc_range, vin_adc_range, Vin_nom, reserved.
if ~isnumeric(CP) || numel(CP) ~= 32
    error('emu_decode_cp:size', 'CP must be a numeric 1x32 vector');
end
CP = double(CP(:)');
names = {'fcsmpc', 'cascade_pi', 'cascade_pid', 'smc', 'single_pi'};
nreq = [1 2 4 2 2];
ffn = {'oracle', 'measured', 'none'};
cid = CP(1);
fid = CP(6);
if cid ~= round(cid) || cid < 1 || cid > 5 || fid ~= round(fid) || fid < 1 || fid > 3
    error('emu_decode_cp:id', 'invalid controller id %g or ff_mode id %g', cid, fid);
end
s.ctrl = names{cid};
s.cid = cid;
s.gains = CP(2:1 + nreq(cid));
s.ff_mode = ffn{fid};
s.delay = CP(7);
s.adc_bits = CP(8);
s.KPV = CP(9);
s.KIV = CP(10);
s.IL_MAX = CP(11);
s.Vref = CP(12);
s.D_ss = CP(13);
s.DMIN = CP(14);
s.DMAX = CP(15);
s.L_model = CP(16);
s.Ts = CP(17);
s.Vo_adc_range = CP(18:19);
s.iL_adc_range = CP(20:21);
s.io_adc_range = CP(22:23);
s.vin_adc_range = CP(24:25);
s.Vin_nom = CP(26);
s.reserved = CP(27:32);
end
