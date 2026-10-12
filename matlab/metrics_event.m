function m = metrics_event(t, Vo, P, ev)
%METRICS_EVENT  Corrected start-up and disturbance metrics (replaces the legacy set).
%   m = metrics_event(t, Vo, P, ev)
%   t, Vo : sample times and output-voltage samples (e.g. out.t, out.Vo)
%   P     : boost_params() (uses Vref, Ts, t_step)
%   ev    : struct, all fields optional
%     t_evt        event time [s] (default P.t_step).  The first post-event
%                  sample is the first t >= t_evt (same test as the profiles).
%     band_pct     settling band, +/- % of Vref (default 2)
%     pre_window   length [s] of the pre-event and end-of-run windows (5e-3)
%     post_window  length [s] analysed after t_evt (default Inf = to the end)
%     Vref         reference (default P.Vref)
%     Vo_min, Vo_max  optional per-period envelope from the switched plant
%                  (out.Vo_min / out.Vo_max, element k = extreme during the
%                  period that starts at t(k), NaN ignored).  When given,
%                  ripple and peak values use the envelope (what a scope
%                  shows); settling/recovery always use the samples.
%
%   Definitions (band: Vref*(1-b) <= Vo <= Vref*(1+b), b = band_pct/100;
%   pre = samples with t < t_evt; post = t_evt <= t <= t_evt + post_window):
%   startup_settling_ms  time from t(1) to the first sample after which every
%                        pre-event sample stays inside the band.  NaN if the
%                        last pre-event sample is outside (never settled; no
%                        sentinel value as in the legacy metric).
%   startup_overshoot_pct max(0, max(Vo_pre) - Vref)/Vref*100
%   pre_offset_V         |mean(Vo) - Vref| over the pre_window before t_evt:
%                        the DC regulation error (signed mean, not mean |e|)
%   pre_ripple_pp_V      max - min over the same window.  Without an envelope
%                        this is only the slow sample-to-sample variation; it
%                        is NOT switching ripple for the averaged plant.
%   event_peak_dev_V     max |Vo - Vref| over post
%   event_peak_time_ms   time of that peak after t_evt
%   event_undershoot_V   max(0, Vref - min(Vo_post))
%   event_overshoot_V    max(0, max(Vo_post) - Vref)
%   event_iae_Vms        integral of |Vo - Vref| over post, in V*ms (Ts*sum)
%   recovery_ms          time after t_evt until Vo re-enters the band and
%                        stays in it until the end of post (0 if it never
%                        leaves the band, NaN if it is outside at the end)
%   post_offset_V        |mean(Vo) - Vref| over the last pre_window of post
%   post_ripple_pp_V     max - min over that window (envelope if given)
%   startup_settled, recovered  logical flags; settled_flag = both true
%   ripple_from_envelope true if Vo_min/Vo_max were used
if nargin < 4 || isempty(ev)
    ev = struct();
end
t = t(:);
Vo = Vo(:);
Ts = P.Ts;
t_evt = getf(ev, 't_evt', P.t_step);
band_pct = getf(ev, 'band_pct', 2.0);
pre_window = getf(ev, 'pre_window', 5e-3);
post_window = getf(ev, 'post_window', Inf);
Vref = getf(ev, 'Vref', P.Vref);
env = isfield(ev, 'Vo_min') && isfield(ev, 'Vo_max') && ~isempty(ev.Vo_min) && ~isempty(ev.Vo_max);
if env
    vmin = ev.Vo_min(:);
    vmax = ev.Vo_max(:);
else
    vmin = Vo;
    vmax = Vo;
end

N = numel(Vo);
Vlo = Vref * (1 - band_pct / 100);
Vhi = Vref * (1 + band_pct / 100);
inb = (Vo >= Vlo) & (Vo <= Vhi);
npre = max(1, round(pre_window / Ts));
tol = 1e-6 * Ts;

i_evt = find(t >= t_evt - tol, 1);
if isempty(i_evt)
    i_evt = N + 1;                       % event after the end of the run
end
ipre = 1:(i_evt - 1);
ipost = find(t >= t_evt - tol & t <= t_evt + post_window + tol);

m.t_evt = t_evt;
m.band_pct = band_pct;

% start-up
if isempty(ipre)
    m.startup_settling_ms = NaN;
    m.startup_overshoot_pct = NaN;
else
    last_out = find(~inb(ipre), 1, 'last');
    if isempty(last_out)
        m.startup_settling_ms = 0;
    elseif last_out == numel(ipre)
        m.startup_settling_ms = NaN;
    else
        m.startup_settling_ms = (t(ipre(last_out + 1)) - t(1)) * 1e3;
    end
    m.startup_overshoot_pct = max(0, (max(vmax(ipre)) - Vref) / Vref * 100);
end

% pre-event steady state
iw = max(1, i_evt - npre):(i_evt - 1);
[m.pre_offset_V, m.pre_ripple_pp_V] = window_stats(Vo, vmin, vmax, iw, Vref);

% event response
if isempty(ipost)
    m.event_peak_dev_V = NaN;
    m.event_peak_time_ms = NaN;
    m.event_undershoot_V = NaN;
    m.event_overshoot_V = NaN;
    m.event_iae_Vms = NaN;
    m.recovery_ms = NaN;
    m.post_offset_V = NaN;
    m.post_ripple_pp_V = NaN;
else
    hi = vmax(ipost);
    lo = vmin(ipost);
    hi(isnan(hi)) = Vo(ipost(isnan(hi)));
    lo(isnan(lo)) = Vo(ipost(isnan(lo)));
    dev = max(hi - Vref, Vref - lo);
    [m.event_peak_dev_V, ipk] = max(dev);
    m.event_peak_time_ms = (t(ipost(ipk)) - t_evt) * 1e3;
    m.event_undershoot_V = max(0, Vref - min(lo));
    m.event_overshoot_V = max(0, max(hi) - Vref);
    m.event_iae_Vms = Ts * sum(abs(Vo(ipost) - Vref)) * 1e3;
    last_out = find(~inb(ipost), 1, 'last');
    if isempty(last_out)
        m.recovery_ms = 0;
    elseif last_out == numel(ipost)
        m.recovery_ms = NaN;
    else
        m.recovery_ms = (t(ipost(last_out + 1)) - t_evt) * 1e3;
    end
    iw = ipost(max(1, numel(ipost) - npre + 1):end);
    [m.post_offset_V, m.post_ripple_pp_V] = window_stats(Vo, vmin, vmax, iw, Vref);
end

m.startup_settled = ~isnan(m.startup_settling_ms);
m.recovered = ~isnan(m.recovery_ms);
m.settled_flag = m.startup_settled && m.recovered;
m.ripple_from_envelope = env;
end

function [off, rip] = window_stats(Vo, vmin, vmax, iw, Vref)
if isempty(iw)
    off = NaN;
    rip = NaN;
    return;
end
off = abs(mean(Vo(iw)) - Vref);
hi = max(vmax(iw));          % max/min ignore NaN
lo = min(vmin(iw));
if isempty(hi) || isnan(hi)
    hi = max(Vo(iw));
end
if isempty(lo) || isnan(lo)
    lo = min(Vo(iw));
end
rip = hi - lo;
end

function v = getf(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
