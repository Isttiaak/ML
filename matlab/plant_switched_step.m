function [iL1, Vo1, info, wave] = plant_switched_step(iL, Vo, d, R, Vin_k, P, nsub)
%PLANT_SWITCHED_STEP  One PWM period of the switched (cycle-resolved) boost.
%   [iL1, Vo1, info] = plant_switched_step(iL, Vo, d, R, Vin_k, P, nsub)
%   [iL1, Vo1, info, wave] = plant_switched_step(...)   also returns the
%                                                       sampled waveform
%
%   Advances exactly one control period P.Ts, which starts at the sampling
%   instant t = 0.  PWM alignment is selected by P.pwm:
%     'trailing' (default, also when the field is missing): trailing-edge
%        PWM, switch ON for t in [0, d*Ts), OFF for [d*Ts, Ts).  A sample at
%        t = 0 sees the ripple VALLEY of iL (bias -dI/2 in CCM).
%     'center' (or 'centre'): centre-aligned PWM (up-down carrier whose
%        valley is at t = 0): ON for [0, d*Ts/2), OFF for [d*Ts/2, Ts -
%        d*Ts/2), ON for [Ts - d*Ts/2, Ts).  t = 0 is the MIDDLE of an ON
%        pulse, so in CCM the sample at t = 0 equals the period average of
%        iL (no dI/2 bias).  The ON pulse straddles the period boundary:
%        its second half [Ts - d*Ts/2, Ts) uses the duty d of THIS call,
%        the first half of the next period uses the next call's duty, so a
%        duty change makes the pulse around k*Ts asymmetric (width
%        (d_k + d_k+1)*Ts/2, centre shifted by (d_k+1 - d_k)*Ts/4).
%   Numeric P.pwm is also accepted (0 = trailing, nonzero = centre).
%     iL     inductor current [A]; clamped to >= 0 unless P.sync is true
%     Vo     CAPACITOR voltage vC [V] (state).  Terminal voltage incl. the
%            ESR drop is returned in info.Vout_end.
%     d      duty in [0,1] (clamped); FCS-MPC passes its gate u = 0 or 1
%     R      load [ohm], R > 0 (Inf = open circuit is allowed)
%     Vin_k  input voltage [V], > 0
%     P      struct with L, C, Ts and optional rL (inductor DCR), rC (cap
%            ESR), Ron (switch), VF (diode drop); missing parasitics = 0.
%            Optional P.sync (default false): synchronous boost, i.e. the
%            diode is replaced by a high-side switch with resistance Ron
%            (VF ignored), current may reverse and there is no DCM.  The
%            legacy averaged model (plant_avg_step) lets iL go negative,
%            so it corresponds to sync = true.  Optional P.pwm (above).
%     nsub   sample points per period (default 100), split between ON and
%            OFF in proportion to d, at least 1 in each non-empty interval
%            (centre PWM: the ON points are split between the two halves)
%   Returns iL1, Vo1 = states at t = Ts and info with fields
%     Vout_end  terminal voltage at t = Ts- (topology active before the end;
%               centre PWM with d > 0: switch ON, Vout = kR*vC)
%     iL_min, iL_max, Vout_min, Vout_max   extremes over the sample points
%               (both sides of the ESR step at a switching instant included)
%     iL_avg, Vout_avg, vC_avg   period averages (trapezoid on the points)
%     dcm       true if the diode current reached 0 (inductor idle)
%     t_on      d*Ts (total ON time in both modes);  t_idle  time spent in
%               the idle (DCM) topology
%     iL_mid_on, Vout_mid_on   value at the centre of the ON pulse, where a
%               DSP usually samples (in CCM iL there equals the period
%               average).  Trailing: at t = t_on/2 (iL at t = 0 is the
%               valley).  Centre: at t = 0, i.e. the start sample itself
%               (the input iL after clamping, Vout = kR*Vo).
%   wave (only computed when requested) has column vectors t (relative to
%   the period start, 0..Ts), iL, vC, Vout at all sample points; switching
%   instants appear twice (before/after), so use it with trapz or
%   ripple_from_waveform, not with interp1.
%
%   Circuit: Vin - L (DCR rL) - switch node; switch (Ron) node->ground when
%   ON; diode (drop VF, ideal otherwise) node->output when OFF; output = C
%   with ESR rC in series, in parallel with R.  With iD the diode current,
%   kR = R/(R+rC) and g = 1/(R+rC):
%     Vout = kR*(vC + rC*iD)      iC = C dvC/dt = kR*iD - g*vC
%   ON   (iD = 0):     L diL/dt = Vin - (rL + Ron)*iL
%                      C dvC/dt = -g*vC                 Vout = kR*vC
%   OFF, conducting (iD = iL > 0):
%                      L diL/dt = Vin - VF - (rL + kR*rC)*iL - kR*vC
%                      C dvC/dt = kR*iL - g*vC          Vout = kR*(vC + rC*iL)
%   OFF, idle / DCM (iL = 0, diode blocks while Vout > Vin - VF):
%                      diL/dt = 0,  C dvC/dt = -g*vC    Vout = kR*vC
%   sync = true: OFF is always 'conducting' with rL -> rL + Ron and VF = 0.
%   With rC = 0: kR = 1, g = 1/R and Vout = vC, i.e. the averaged model of
%   plant_avg_step is recovered by weighting ON/OFF with d and 1-d.
%   DCM can only start inside the OFF interval (in both PWM modes); with
%   centre PWM an idle inductor restarts from iL = 0 in the second ON half.
%
%   Integration: each topology is linear time-invariant, so it is solved
%   EXACTLY (closed-form 2x2 matrix exponential, x(t) = xeq + e^(A t)(x0-xeq),
%   e^(A t) = e^(s t)[c(t) I + S(t)(A - s I)]).  There is no truncation
%   error: nsub only sets the sampling of min/max/averages and of the DCM
%   detection.  The instant where iL reaches 0 in the OFF state is found by
%   safeguarded Newton on the analytic solution; re-conduction from the
%   idle state is solved analytically.  This was preferred to RK4 because it
%   is exact for any step, cheap (a few exp/cos/sin per period) and fully
%   vectorised over the sample points.
%   Assumptions: the switch is ideal apart from Ron, the diode does not
%   conduct while the switch is ON (needs Ron*iL < Vout + VF), no reverse
%   recovery, no dead time.
%   sw_boost_period.m is the scalar, code-generation version (Simulink
%   MATLAB Function block) of the same model (15th input pwm_center).

if nargin < 7 || isempty(nsub)
    nsub = 100;
end
L = P.L;
C = P.C;
Ts = P.Ts;
rL = 0; rC = 0; Ron = 0; VF = 0;
if isfield(P, 'rL'), rL = P.rL; end
if isfield(P, 'rC'), rC = P.rC; end
if isfield(P, 'Ron'), Ron = P.Ron; end
if isfield(P, 'VF'), VF = P.VF; end
sync = isfield(P, 'sync') && P.sync;
center = false;
if isfield(P, 'pwm') && ~isempty(P.pwm)
    if ischar(P.pwm)
        switch lower(P.pwm)
            case {'center', 'centre'}
                center = true;
            case 'trailing'
                center = false;
            otherwise
                error('plant_switched_step:pwm', 'P.pwm must be ''trailing'' or ''center''');
        end
    else
        center = P.pwm ~= 0;
    end
end

nsub = max(1, round(nsub));
d = min(max(d, 0), 1);
if ~sync
    iL = max(iL, 0);
end
vC = Vo;
kR = 1 / (1 + rC / R);      % R/(R+rC), = 1 for R = Inf
gC = 1 / ((R + rC) * C);    % g/C
t_on = d * Ts;
t_off = Ts - t_on;
if t_on > 0 && t_off > 0
    n_on = min(max(round(nsub * d), 1), max(nsub - 1, 1));
    n_off = max(nsub - n_on, 1);
elseif t_on > 0
    n_on = nsub; n_off = 0;
else
    n_on = 0; n_off = nsub;
end

dcm = false;
t_idle = 0;
iLm = iL;
Vm = NaN;
a = -(rL + Ron) / L;              % ON: diL/dt = a*iL + Vin/L
VinL = Vin_k / L;

% ---------------- ON interval(s): decoupled first-order equations -------
% All sample points of the period are collected in T (time), I (iL),
% V (vC) and O (Vout) and reduced once at the end.  A switching instant
% appears twice in T (zero-width trapezoid) so that the ESR step in Vout is
% represented on both sides.
if n_on > 0 && ~center
    % trailing edge: one ON interval [0, t_on)
    [T, I, V] = sw_on_seg(iL, vC, t_on, n_on, a, VinL, gC);
    O = kR * V;
    tm = 0.5 * t_on;
    if a == 0
        iLm = iL + VinL * tm;
    else
        iLm = iL * exp(a * tm) + VinL * (expm1(a * tm) / a);
    end
    Vm = kR * vC * exp(-gC * tm);
    iL = I(end);
    vC = V(end);
    t_off0 = t_on;                % OFF interval starts here
elseif n_on > 0
    % centre aligned: first ON half [0, t_on/2); iL_mid_on = start sample
    n_a = ceil(n_on / 2);
    n_b = max(n_on - n_a, 1);
    t_half = 0.5 * t_on;
    [T, I, V] = sw_on_seg(iL, vC, t_half, n_a, a, VinL, gC);
    O = kR * V;
    iL = I(end);
    vC = V(end);
    t_off0 = t_half;
else
    T = []; I = []; V = []; O = [];
    t_off0 = 0;
end

% ---------------- OFF interval: conducting / idle segments ---------------
if n_off > 0
    h = t_off / n_off;
    % conducting-topology constants: A = [a11 a12; a21 a22], b = [b1; 0]
    if sync
        rOff = rL + Ron;
        vth = Vin_k;
    else
        rOff = rL;
        vth = Vin_k - VF;             % diode conducts while Vout < vth
    end
    a11 = -(rOff + kR * rC) / L;
    a12 = -kR / L;
    a21 = kR / C;
    s = 0.5 * (a11 - gC);
    m11 = 0.5 * (a11 + gC);           % A - s*I = [m11 a12; a21 -m11]
    Dl = m11 * m11 + a12 * a21;       % (A - s*I)^2 = Dl*I
    iLeq = vth / (rOff + R);
    vCeq = vth / (1 + rOff / R);

    ts = 0;
    idle = ~sync && (iL <= 0) && (vth - kR * vC < 0);
    nseg = 0;
    while true
        nseg = nseg + 1;
        events_on = ~sync && nseg < 8;   % bounded number of mode changes
        j0 = floor(ts / h) + 1;
        tau = [0, (j0:n_off - 1) * h - ts, t_off - ts];
        if numel(tau) > 2 && tau(2) <= 0
            tau(2) = [];
        end
        event = false;
        if ~idle
            di0 = iL - iLeq;
            dv0 = vC - vCeq;
            e = exp(s * tau);
            if Dl < 0
                w = sqrt(-Dl);
                c = cos(w * tau);
                S = sin(w * tau) / w;
            elseif Dl > 0
                w = sqrt(Dl);
                c = cosh(w * tau);
                S = sinh(w * tau) / w;
            else
                c = ones(size(tau));
                S = tau;
            end
            iv = iLeq + e .* (c * di0 + S * (m11 * di0 + a12 * dv0));
            vv = vCeq + e .* (c * dv0 + S * (a21 * di0 - m11 * dv0));
            iv(1) = iL;
            vv(1) = vC;
            if events_on
                kneg = find(iv < 0, 1);
                if ~isempty(kneg)
                    K = [s, m11, a12, a21, Dl, iLeq, vCeq, a11, vth / L];
                    tz = sw_ccm_zero(tau(kneg - 1), iv(kneg - 1), tau(kneg), iv(kneg), ...
                        di0, dv0, K);
                    [~, vz] = sw_ccm_eval(tz, di0, dv0, K);
                    tau = [tau(1:kneg - 1), tz];
                    iv = [iv(1:kneg - 1), 0];
                    vv = [vv(1:kneg - 1), vz];
                    event = true;
                    dcm = true;
                end
            end
            ov = kR * (vv + rC * iv);
        else
            if vth <= 0
                tstar = Inf;
            elseif kR * vC <= vth
                tstar = 0;
            elseif gC > 0
                tstar = log(kR * vC / vth) / gC;
            else
                tstar = Inf;
            end
            if events_on && tstar < tau(end)
                kk = find(tau >= tstar, 1);
                tau = [tau(1:kk - 1), tstar];
                event = true;
            end
            vv = vC * exp(-gC * tau);
            iv = zeros(size(tau));
            ov = kR * vv;
            dcm = true;
            t_idle = t_idle + tau(end);
        end
        T = [T, (t_off0 + ts) + tau]; %#ok<AGROW>
        I = [I, iv]; %#ok<AGROW>
        V = [V, vv]; %#ok<AGROW>
        O = [O, ov]; %#ok<AGROW>
        iL = iv(end);
        vC = vv(end);
        if ~event
            break;
        end
        ts = ts + tau(end);
        if ts >= t_off
            break;
        end
        idle = ~idle;
    end
end

% ---------------- centre aligned: second ON half [Ts - t_on/2, Ts) -------
if n_on > 0 && center
    [T2, I2, V2] = sw_on_seg(iL, vC, t_half, n_b, a, VinL, gC);
    T = [T, (t_half + t_off) + T2];
    I = [I, I2];
    V = [V, V2];
    O = [O, kR * V2];
    iL = I2(end);
    vC = V2(end);
end

% trapezoid weights on the (non-uniform) sample times
dT = diff(T);
wt = 0.5 * ([dT, 0] + [0, dT]);
iL1 = iL;
Vo1 = vC;
if n_on == 0 || center
    Vm = O(1);                        % sample at t = 0 (d = 0: OFF state)
end
info = struct('Vout_end', O(end), 'iL_min', min(I), 'iL_max', max(I), ...
    'Vout_min', min(O), 'Vout_max', max(O), 'iL_avg', (I * wt.') / Ts, ...
    'Vout_avg', (O * wt.') / Ts, 'vC_avg', (V * wt.') / Ts, 'dcm', dcm, ...
    't_on', t_on, 't_idle', t_idle, 'iL_mid_on', iLm, 'Vout_mid_on', Vm);
if nargout > 3
    wave = struct('t', T(:), 'iL', I(:), 'vC', V(:), 'Vout', O(:));
end
end

function [T, I, V] = sw_on_seg(iL, vC, tlen, n, a, VinL, gC)
% switch ON for tlen from (iL, vC), n + 1 equally spaced points
T = (0:n) * (tlen / n);
if a == 0
    I = iL + VinL * T;
else
    I = iL * exp(a * T) + VinL * (expm1(a * T) / a);
end
V = vC * exp(-gC * T);
end

function [iv, vv] = sw_ccm_eval(tau, di0, dv0, K)
% closed-form conducting-OFF solution at times tau (row) from (di0, dv0)
% relative to the equilibrium; K = [s m11 a12 a21 Dl iLeq vCeq a11 b1]
s = K(1); m11 = K(2); a12 = K(3); a21 = K(4); Dl = K(5);
e = exp(s * tau);
if Dl < 0
    w = sqrt(-Dl);
    c = cos(w * tau);
    S = sin(w * tau) / w;
elseif Dl > 0
    mu = sqrt(Dl);
    c = cosh(mu * tau);
    S = sinh(mu * tau) / mu;
else
    c = ones(size(tau));
    S = tau;
end
p = m11 * di0 + a12 * dv0;
q = a21 * di0 - m11 * dv0;
iv = K(6) + e .* (c * di0 + S * p);
vv = K(7) + e .* (c * dv0 + S * q);
end

function tz = sw_ccm_zero(ta, fa, tb, fb, di0, dv0, K)
% first zero of iL(t) in [ta, tb] with iL(ta) >= 0 > iL(tb):
% Newton on the analytic solution, bisection when Newton leaves the bracket
a11 = K(8); a12 = K(3); b1 = K(9);
tol = 1e-13 * (tb - ta) + 1e-300;
if fa <= 0
    tz = ta;
    return;
end
t = ta - fa * (tb - ta) / (fb - fa);
for it = 1:60
    [f, v] = sw_ccm_eval(t, di0, dv0, K);
    if f == 0
        break;
    elseif f > 0
        ta = t;
    else
        tb = t;
    end
    fp = a11 * f + a12 * v + b1;
    tn = t - f / fp;
    if ~(tn > ta && tn < tb)
        tn = 0.5 * (ta + tb);
    end
    if abs(tn - t) <= tol || (tb - ta) <= tol
        t = tn;
        break;
    end
    t = tn;
end
tz = t;
end
