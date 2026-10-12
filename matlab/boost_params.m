function P = boost_params()
%BOOST_PARAMS  Legacy constants of the Python/Colab boost-converter study.
%   P = boost_params() returns a struct with the exact values used in the
%   notebook (cell 1).  Integer sample counts replicate Python int(x/Ts),
%   i.e. fix() applied to the same double expression (note that
%   fix(0.075/Ts) is 1499, not 1500).
%
%   Plant / control constants (legacy):
%     Vin Vref L C R0 R1 fs Ts T_end t_step N D_ss DMIN DMAX KPV KIV IL_MAX
%   Parasitics (used only by the switched plant, all 0 = ideal legacy):
%     rL  inductor DCR [ohm]     rC  capacitor ESR [ohm]
%     Ron switch on-resistance   VF  diode forward drop [V]
%   ADC full-scale ranges [lo hi] used by sim_boost when adc_bits > 0:
%     Vo_adc_range        [0 60]  V  (as in the notebook)
%     iL_adc_range        [0 18]  A  = 1.5*IL_MAX, so that current overshoot
%                                    above IL_MAX is not clipped by the ADC
%     iL_adc_range_legacy [0 12]  A  = notebook value (0..IL_MAX); pass it as
%                                    opts.iL_adc_range to reproduce cell 12
%     io_adc_range        [0 15]  A  output-current sensor (max load 48V/5ohm
%                                    = 9.6 A plus margin)
%     vin_adc_range       [0 60]  V  input-voltage sensor
%   Derived: k_step (sample index of the load step, Python int(t_step/Ts)),
%   IL_ff0/IL_ff1 (legacy oracle feed-forward before/after the step).
%   gains_baseline: default (hand-tuned) gains of the notebook sim_* functions,
%   in the order expected by sim_boost.

P.Vin    = 24.0;
P.Vref   = 48.0;
P.L      = 500e-6;
P.C      = 470e-6;
P.R0     = 20.0;
P.R1     = 10.0;
P.fs     = 20e3;
P.Ts     = 1.0 / P.fs;
P.T_end  = 0.100;
P.t_step = 0.050;
P.N      = fix(P.T_end / P.Ts);
P.k_step = fix(P.t_step / P.Ts);

P.D_ss   = 1.0 - P.Vin / P.Vref;
P.DMIN   = 0.05;
P.DMAX   = 0.90;

P.KPV    = 0.15;
P.KIV    = 12.0;
P.IL_MAX = 12.0;

P.IL_ff0 = (P.Vref * P.Vref) / (P.Vin * P.R0);
P.IL_ff1 = (P.Vref * P.Vref) / (P.Vin * P.R1);

% parasitics (switched plant only)
P.rL  = 0;
P.rC  = 0;
P.Ron = 0;
P.VF  = 0;

% sensor / ADC ranges
P.Vo_adc_range        = [0 60];
P.iL_adc_range        = [0 1.5 * P.IL_MAX];
P.iL_adc_range_legacy = [0 P.IL_MAX];
P.io_adc_range        = [0 15];
P.vin_adc_range       = [0 60];

% notebook default gains (order = sim_boost gains vector)
P.gains_baseline.fcsmpc      = 0.5;                          % lsw
P.gains_baseline.cascade_pi  = [0.030 20.0];                 % Kp_i Ki_i
P.gains_baseline.cascade_pid = [0.033 22.0 8e-6 1e-4];       % Kp_i Ki_i Kd_i Tf
P.gains_baseline.smc         = [0.40 0.50];                  % K_s phi
P.gains_baseline.single_pi   = [0.05 5.0];                   % Kp_v Ki_v
end
