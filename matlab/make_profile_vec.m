function PV = make_profile_vec(spec, which, P)
%MAKE_PROFILE_VEC  1x7 profile vector (block Parameter LP or VP) from a spec struct.
%   PV = make_profile_vec(spec, which, P)
%     spec  load_profile / vin_profile spec struct ([] = sim_boost default)
%     which 'load' | 'vin'
%     P     boost_params() struct ([] = default)
%   PV = [kind p1 .. p6] as evaluated by blk_profile_eval:
%     0 const       [0 v]
%     1 step        [1 t_chg v_before v_after]
%     2 ramp        [2 t0 t1 v0 v_end]
%     3 double_step [3 t1 t2 v0 v_mid v_end]          (load only)
%   Defaults are exactly those of sim_boost + load_profile / vin_profile:
%     load: [] -> legacy step R0 = P.R0 -> R_new = P.R1 at P.t_step;
%           missing R0 -> P.R0; step pct 0.5, t_step 0.050,
%           R_new = R0*(1 - pct); ramp t0 0.050, t1 0.060, R_end 10;
%           double_step t1 0.050, R_mid 14, t2 0.075, R_end 10; const R = R0.
%     vin : [] -> const; missing Vin -> P.Vin; step pct 0.20, t_step 0.060,
%           V_new = Vin*(1 + pct); ramp t0 0.060, t1 0.070, V_end = Vin*1.2.

if nargin < 3 || isempty(P)
    P = boost_params();
end
PV = zeros(1, 7);
switch lower(which)
    case 'load'
        if isempty(spec)
            spec = struct('kind', 'step', 'R0', P.R0, 'R_new', P.R1, 't_step', P.t_step);
        end
        if ~isfield(spec, 'R0')
            spec.R0 = P.R0;
        end
        kind = lower(getf(spec, 'kind', 'step'));
        R0 = getf(spec, 'R0', 20.0);
        switch kind
            case 'const'
                PV(1:2) = [0, getf(spec, 'R', R0)];
            case 'step'
                pct = getf(spec, 'pct', 0.5);
                t_chg = getf(spec, 't_step', 0.050);
                R_new = getf(spec, 'R_new', R0 * (1 - pct));
                PV(1:4) = [1, t_chg, R0, R_new];
            case 'ramp'
                PV(1:5) = [2, getf(spec, 't0', 0.050), getf(spec, 't1', 0.060), R0, ...
                    getf(spec, 'R_end', 10.0)];
            case 'double_step'
                PV(1:6) = [3, getf(spec, 't1', 0.050), getf(spec, 't2', 0.075), R0, ...
                    getf(spec, 'R_mid', 14.0), getf(spec, 'R_end', 10.0)];
            otherwise
                error('make_profile_vec:kind', 'unknown load profile kind ''%s''', kind);
        end
    case 'vin'
        if isempty(spec)
            spec = struct('kind', 'const');
        end
        if ~isfield(spec, 'Vin')
            spec.Vin = P.Vin;
        end
        kind = lower(getf(spec, 'kind', 'const'));
        Vin = getf(spec, 'Vin', 24.0);
        switch kind
            case 'const'
                PV(1:2) = [0, Vin];
            case 'step'
                pct = getf(spec, 'pct', 0.20);
                t_chg = getf(spec, 't_step', 0.060);
                V_new = getf(spec, 'V_new', Vin * (1 + pct));
                PV(1:4) = [1, t_chg, Vin, V_new];
            case 'ramp'
                PV(1:5) = [2, getf(spec, 't0', 0.060), getf(spec, 't1', 0.070), Vin, ...
                    getf(spec, 'V_end', Vin * 1.2)];
            otherwise
                error('make_profile_vec:kind', 'unknown Vin profile kind ''%s''', kind);
        end
    otherwise
        error('make_profile_vec:which', 'which must be ''load'' or ''vin''');
end
end

function v = getf(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
