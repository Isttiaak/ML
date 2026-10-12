function V = vin_profile(t, spec)
%VIN_PROFILE  Input voltage Vin(t) [V] for a profile spec (vectorised in t).
%   V = vin_profile(t, spec)   t = k*Ts, spec a struct:
%     kind 'const' : Vin (24)
%     kind 'step'  : Vin (24), pct (0.20), t_step (0.060); V_new = Vin*(1+pct)
%                    unless spec.V_new is given.  V = V_new for t >= t_step.
%     kind 'ramp'  : Vin (24), t0, t1, V_end - linear change (not in notebook).
%   Defaults follow make_vin_profile() in the notebook (cell 12).
if nargin < 2 || isempty(spec)
    spec = struct('kind', 'const');
end
kind = lower(getf(spec, 'kind', 'const'));
Vin = getf(spec, 'Vin', 24.0);
V = Vin + zeros(size(t));
switch kind
    case 'const'
        % nothing to do
    case 'step'
        pct = getf(spec, 'pct', 0.20);
        t_chg = getf(spec, 't_step', 0.060);
        V_new = getf(spec, 'V_new', Vin * (1 + pct));
        V(t >= t_chg) = V_new;
    case 'ramp'
        t0 = getf(spec, 't0', 0.060);
        t1 = getf(spec, 't1', 0.070);
        V_end = getf(spec, 'V_end', Vin * 1.2);
        mid = (t >= t0) & (t <= t1);
        V(mid) = Vin + (t(mid) - t0) / (t1 - t0) * (V_end - Vin);
        V(t > t1) = V_end;
    otherwise
        error('vin_profile:kind', 'unknown Vin profile kind ''%s''', kind);
end
end

function v = getf(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
