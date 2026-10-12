function m = metrics_legacy(Vo, P, t_evt, o)
%METRICS_LEGACY  Exact replica of the notebook metrics (kept for reproduction).
%   m = metrics_legacy(Vo, P)          -> compute_metrics()   (cell 1)
%   m = metrics_legacy(Vo, P, t_evt)   -> metrics_scenario()  (cell 12)
%   m = metrics_legacy(Vo, P, t_evt, o) with optional o.band_pct (2),
%       o.ss_window ([0.040 0.050]), o.lr_window ([t_evt min(t_evt+0.025,T_end)])
%
%   Fields: settling_ms overshoot_pct ss_error_V ripple_V load_reg_V
%   Windows are sample slices Vo(k1+1:k2) with k = fix(t/Ts) as Python int().
%   Rounding replicates Python exactly: settling/ss/ripple/load_reg use the
%   built-in round(float, n) (py_round); overshoot uses numpy's round on a
%   np.float64 (rint(x*1e4)/1e4).  The mean uses numpy pairwise summation.
%
%   Known flaws (see metrics_event for the corrected versions): settling
%   returns t_step (a sentinel 50 ms) when never settled; the 'SS error' and
%   'ripple' windows (40-50 ms) can still be inside the start-up transient;
%   ripple of the averaged model is not switching ripple; the load-reg window
%   for compute_metrics is 50 ms .. 74.95 ms (int(0.075/Ts) = 1499).
if nargin < 4 || isempty(o)
    o = struct();
end
Ts = P.Ts;
Vref = P.Vref;
Vo = Vo(:);
band_pct = getf(o, 'band_pct', 2.0);
ssw = getf(o, 'ss_window', [0.040 0.050]);
if nargin < 3 || isempty(t_evt)
    k_evt = fix(P.t_step / Ts);
    lrw = getf(o, 'lr_window', [0.050 0.075]);
else
    k_evt = fix(t_evt / Ts);
    lrw = getf(o, 'lr_window', [t_evt, min(t_evt + 0.025, P.T_end)]);
end
k1 = fix(ssw(1) / Ts);
k2 = fix(ssw(2) / Ts);
l1 = fix(lrw(1) / Ts);
l2 = fix(lrw(2) / Ts);

% settling: first k such that Vo(k:k_evt) stays inside the band
Vlo = Vref * (1 - band_pct / 100);
Vhi = Vref * (1 + band_pct / 100);
seg = Vo(1:k_evt);
inb = (seg >= Vlo) & (seg <= Vhi);
last_out = find(~inb, 1, 'last');
if isempty(last_out)
    ks = 0;
else
    ks = last_out;               % python index of the sample after last_out
end
m.settling_ms = py_round(ks * Ts * 1e3, 3);

% overshoot (numpy max propagates NaN, max(0.0, nan) returns 0.0)
pk = npmax(seg);
x = (pk - Vref) / Vref * 100;
if x > 0
    m.overshoot_pct = core_np_round(x, 4);
else
    m.overshoot_pct = 0.0;
end

sw = Vo(k1 + 1:k2);
m.ss_error_V = py_round(core_np_sum(abs(sw - Vref)) / numel(sw), 4);
m.ripple_V = py_round(npmax(sw) - npmin(sw), 4);
lw = Vo(l1 + 1:l2);
m.load_reg_V = py_round(npmax(abs(lw - Vref)), 4);
end

function v = npmax(x)
if any(isnan(x))
    v = NaN;
else
    v = max(x);
end
end

function v = npmin(x)
if any(isnan(x))
    v = NaN;
else
    v = min(x);
end
end

function v = getf(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
