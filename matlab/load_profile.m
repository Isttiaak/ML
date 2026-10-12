function R = load_profile(t, spec)
%LOAD_PROFILE  Load resistance R(t) [ohm] for a profile spec (vectorised in t).
%   R = load_profile(t, spec)   t = k*Ts (scalar or array), spec a struct:
%     kind 'const'       : R (default R0)
%     kind 'step'        : R0 (20), pct (0.5), t_step (0.050); R_new = R0*(1-pct)
%                          unless spec.R_new is given.  R = R_new for t >= t_step.
%     kind 'ramp'        : R0 (20), t0 (0.050), t1 (0.060), R_end (10);
%                          linear from R0 at t0 to R_end at t1.
%     kind 'double_step' : R0 (20), t1 (0.050), R_mid (14), t2 (0.075), R_end (10).
%   Defaults (in brackets) and comparisons are those of make_load_profile()
%   in the notebook (cell 12), so t must be computed as k*Ts like Python.
%   Note: a 'step' with pct = 0.75 gives R = 5 ohm, which needs
%   IL = Vref^2/(Vin*R) = 19.2 A > IL_MAX = 12 A: infeasible (R >= 8 ohm).
if nargin < 2 || isempty(spec)
    spec = struct('kind', 'step');
end
kind = lower(getf(spec, 'kind', 'step'));
R0 = getf(spec, 'R0', 20.0);
R = R0 + zeros(size(t));
switch kind
    case 'const'
        R(:) = getf(spec, 'R', R0);
    case 'step'
        pct = getf(spec, 'pct', 0.5);
        t_chg = getf(spec, 't_step', 0.050);
        R_new = getf(spec, 'R_new', R0 * (1 - pct));
        R(t >= t_chg) = R_new;
    case 'ramp'
        t0 = getf(spec, 't0', 0.050);
        t1 = getf(spec, 't1', 0.060);
        R_end = getf(spec, 'R_end', 10.0);
        mid = (t >= t0) & (t <= t1);
        frac = (t(mid) - t0) / (t1 - t0);
        R(mid) = R0 + frac * (R_end - R0);
        R(t > t1) = R_end;
    case 'double_step'
        t1 = getf(spec, 't1', 0.050);
        R_mid = getf(spec, 'R_mid', 14.0);
        t2 = getf(spec, 't2', 0.075);
        R_end = getf(spec, 'R_end', 10.0);
        R(t >= t1 & t < t2) = R_mid;
        R(t >= t2) = R_end;
    otherwise
        error('load_profile:kind', 'unknown load profile kind ''%s''', kind);
end
end

function v = getf(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
