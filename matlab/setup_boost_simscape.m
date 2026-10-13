function S = setup_boost_simscape(mdl, ctrl, gains, P, opts)
%SETUP_BOOST_SIMSCAPE  Model-workspace variables of boost_simscape.slx.
%   S = setup_boost_simscape(mdl, ctrl, gains, P, opts)
%   Same arguments as sim_boost.  mdl = '' only returns the struct.
%   The Simscape model has ONE explicit Unit Delay between the controller and
%   the PWM (real DSP latency, and it breaks the algebraic loop), so the
%   total delay is opts.delay (default 1, minimum 1) and CP gets delay - 1.
%   Compare a run with sim_boost(ctrl, gains, P, opts) using
%   opts.plant = 'switched', opts.pwm = 'center' and the same delay.
%   Extra opts for this model:
%     h          Simscape local-solver step (default Ts/500 = 0.1 us; Ts/h
%                must be an integer, PWM edges are quantised to h)
%     open_loop  [] (default, closed loop) or a fixed duty in [0 1]
%     Pplant     plant parameters (L C rL rC Ron VF); switch Ron is at
%                least 1e-3 ohm and Roff 1e6 ohm, diode Ron 1e-4 ohm
%     fine_dec   decimation of the fine log 'yfine' (default 1 = every h)
%   Variables written: CP X0 LP VP Ts N (as for boost_avg) and
%   D0 (initial applied duty), h, Ron_sw, Roff_sw, cpk (carrier peak),
%   Lp Cp rLp rCp Vfp Rdp Gdp iL0p vC0p (power stage), OL, d_ol, fine_dec.

if nargin < 1
    mdl = '';
end
if nargin < 2 || isempty(ctrl)
    ctrl = 'cascade_pi';
end
if nargin < 3
    gains = [];
end
if nargin < 4 || isempty(P)
    P = boost_params();
end
if nargin < 5 || isempty(opts)
    opts = struct();
end

dtot = getf(opts, 'delay', 1);
if dtot < 1
    warning('setup_boost_simscape:delay', ...
        'boost_simscape has a fixed one-sample delay; using delay = 1');
    dtot = 1;
end
o2 = opts;
o2.delay = dtot - 1;
o2.plant = 'avg';
if isfield(o2, 'pwm')
    o2 = rmfield(o2, 'pwm');
end
if isfield(o2, 'Pplant')
    o2 = rmfield(o2, 'Pplant');
end
W = setup_boost_workspace('', ctrl, gains, P, o2);
S.CP = W.CP;
S.X0 = W.X0;
S.LP = W.LP;
S.VP = W.VP;
S.Ts = W.Ts;
S.N = W.N;
S.noise = W.noise;
S.t = W.t;
S.name = W.name;
S.D0 = W.X0(6);                      % D_ss (0 for FCS-MPC), as sim_boost

Pp = getf(opts, 'Pplant', P);
S.h = getf(opts, 'h', P.Ts / 500);
S.Ron_sw = max(getf(Pp, 'Ron', 0), 1e-3);
S.Roff_sw = 1e6;
S.cpk = 1 - 1e-9;                    % carrier peak: a gate of 1 stays ON
S.Lp = Pp.L;
S.Cp = Pp.C;
S.rLp = getf(Pp, 'rL', 0);
S.rCp = max(getf(Pp, 'rC', 0), 1e-6);
S.Vfp = getf(Pp, 'VF', 0);
S.Rdp = 1e-4;
S.Gdp = 1e-8;
S.iL0p = getf(opts, 'iL0', 0);
S.vC0p = getf(opts, 'Vo0', P.Vin);
ol = getf(opts, 'open_loop', []);
S.OL = double(~isempty(ol));
if isempty(ol)
    S.d_ol = 0;
else
    S.d_ol = ol;
end
S.fine_dec = getf(opts, 'fine_dec', 1);
S.delay_total = dtot;

if ~isempty(mdl)
    mws = get_param(mdl, 'ModelWorkspace');
    names = {'CP', 'X0', 'LP', 'VP', 'Ts', 'N', 'D0', 'h', 'Ron_sw', 'Roff_sw', 'cpk', ...
        'Lp', 'Cp', 'rLp', 'rCp', 'Vfp', 'Rdp', 'Gdp', 'iL0p', 'vC0p', 'OL', 'd_ol', 'fine_dec'};
    for i = 1:numel(names)
        assignin(mws, names{i}, S.(names{i}));
    end
end
end

function v = getf(s, name, default)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
