function [pp, avg, info] = ripple_from_waveform(t, y, fs, win)
%RIPPLE_FROM_WAVEFORM  Peak-to-peak ripple and mean over whole PWM periods.
%   [pp, avg, info] = ripple_from_waveform(t, y, fs, win)
%   For finely sampled waveforms, e.g. Simscape / variable-step logs with a
%   non-uniform time grid.
%     t    time vector [s], non-decreasing; repeated time stamps (both
%          sides of a switching event, as variable-step solvers and
%          plant_switched_step's 'wave' output produce) are allowed, and
%          rounding-level reversals (< 1e-9/fs) are treated as repeats.
%          A MATLAB timeseries may be passed as t (y = [] then).
%     y    samples, vector or matrix with numel(t) rows (one signal per
%          column)
%     fs   PWM switching frequency [Hz] (P.fs)
%     win  [t0 t1] analysis window (default: whole record).  The window
%          used is [t0, t0 + n*Tsw] with n = floor((t1 - t0)/Tsw) whole PWM
%          periods (Tsw = 1/fs), clipped to the record, so the mean has no
%          partial-period bias.  Choose t0 after the transient of interest.
%   pp   max - min inside the window (samples plus the linearly
%        interpolated values at the window edges)
%   avg  time-weighted mean, trapz(t, y)/(t_end - t_start), with linear
%        interpolation at the window edges (exact for piecewise-linear y)
%   info struct: n_periods, n_points (log samples inside the window),
%        t_start, t_end, ymin, ymax, min_points_per_period (sparsest
%        period), aliasing (true if that is < 20).
%   A warning 'ripple_from_waveform:aliasing' is issued when some PWM period
%   holds fewer than 20 samples: max - min then underestimates the ripple
%   unless the solver steps exactly on every switching instant (log more
%   finely: smaller max step size, 'Refine output', Simscape logging with
%   decimation 1 and no data-point limit).
%   Example (one period of plant_switched_step, P = boost_params()):
%     [~, ~, ~, w] = plant_switched_step(4.8, 48, 0.5, 20, 24, P, 200);
%     [pp, avg] = ripple_from_waveform(w.t, w.iL, P.fs)

if isa(t, 'timeseries')
    y = t.Data;
    t = t.Time;
    if ndims(y) == 3                      % m x n x N logging layout
        y = reshape(permute(y, [3 1 2]), size(y, 3), []);
    end
end
t = double(t(:));
if isvector(y)
    y = y(:);
end
y = double(y);
if size(y, 1) ~= numel(t)
    error('ripple_from_waveform:size', 'y must have numel(t) rows');
end
Tsw = 1 / fs;
if numel(t) < 2 || any(~isfinite(t)) || any(diff(t) < -1e-9 * Tsw)
    error('ripple_from_waveform:time', 't must be finite, non-decreasing, with >= 2 samples');
end
t = cummax(t);          % rounding-level reversals (concatenated logs) -> repeats
if nargin < 4 || isempty(win)
    win = [t(1), t(end)];
end
t0 = max(win(1), t(1));
t1 = min(win(2), t(end));
np = floor((t1 - t0) / Tsw + 1e-9);      % tolerate rounding of exact multiples
if np < 1
    error('ripple_from_waveform:window', ...
        'window [%g %g] s holds no complete PWM period (Tsw = %g s)', t0, t1, Tsw);
end
ta = t0;
tb = min(t0 + np * Tsw, t(end));

in = (t >= ta) & (t <= tb);
tw = t(in);
yw = y(in, :);
npts = numel(tw);
if npts == 0 || tw(1) > ta
    tw = [ta; tw];
    yw = [lin_at(t, y, ta); yw];
end
if tw(end) < tb
    tw = [tw; tb];
    yw = [yw; lin_at(t, y, tb)];
end

ymax = max(yw, [], 1);
ymin = min(yw, [], 1);
pp = ymax - ymin;
avg = trapz(tw, yw, 1) / (tb - ta);

% samples per PWM period (sparsest one)
tin = t(in);
k = min(floor((tin - ta) / Tsw), np - 1) + 1;
cnt = accumarray(k, 1, [np, 1]);
mpp = min(cnt);
alias = mpp < 20;
if alias
    warning('ripple_from_waveform:aliasing', ...
        ['only %d samples in the sparsest of %d PWM periods (< 20): pk-pk may be ' ...
        'underestimated (aliasing); log the waveform more finely'], mpp, np);
end
info = struct('n_periods', np, 'n_points', npts, 't_start', ta, 't_end', tb, ...
    'ymin', ymin, 'ymax', ymax, 'min_points_per_period', mpp, 'aliasing', alias);
end

function v = lin_at(t, y, tq)
% linear interpolation at tq, which is not a sample time and lies inside
% (t(1), t(end)); with repeated stamps the right-hand value is used
j = find(t > tq, 1);
i = j - 1;
v = y(i, :) + (y(j, :) - y(i, :)) * ((tq - t(i)) / (t(j) - t(i)));
end
