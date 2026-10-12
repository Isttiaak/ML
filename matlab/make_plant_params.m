function PP = make_plant_params(Pplant, opts)
%MAKE_PLANT_PARAMS  1x10 plant parameter vector PP for blk_plant_avg / blk_plant_switched.
%   PP = make_plant_params(Pplant, opts)
%     Pplant  plant parameter struct (boost_params() or a modified copy, e.g.
%             L*1.2, C*0.8, rL, rC, Ron, VF, optional sync); [] = boost_params()
%     opts    sim_boost-style options; fields used:
%               nsub  sample points per period of the switched plant (100)
%               pwm   'trailing' (default) | 'center' ('centre'); numeric
%                     0 / nonzero is accepted too.  Pplant.pwm (the field
%                     read by plant_switched_step) is used when opts.pwm is
%                     absent.
%   PP = [L C Ts rL rC Ron VF nsub sync pwm_center]
%   Missing parasitics are 0 and sync = 0, as in plant_switched_step.
%   blk_plant_avg uses only L, C, Ts.  The Simulink models run at one base
%   rate, so Pplant.Ts must equal the controller P.Ts.

if nargin < 1 || isempty(Pplant)
    Pplant = boost_params();
end
if nargin < 2 || isempty(opts)
    opts = struct();
end
nsub = getf(opts, 'nsub', 100);
pwm = getf(opts, 'pwm', getf(Pplant, 'pwm', 'trailing'));
if ischar(pwm)
    switch lower(pwm)
        case 'trailing'
            pwm_center = 0;
        case {'center', 'centre'}
            pwm_center = 1;
        otherwise
            error('make_plant_params:pwm', 'pwm must be ''trailing'' or ''center''');
    end
else
    pwm_center = double(pwm ~= 0);
end
PP = [Pplant.L, Pplant.C, Pplant.Ts, getf(Pplant, 'rL', 0), getf(Pplant, 'rC', 0), ...
    getf(Pplant, 'Ron', 0), getf(Pplant, 'VF', 0), nsub, double(getf(Pplant, 'sync', false) ~= 0), ...
    pwm_center];
end

function v = getf(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
