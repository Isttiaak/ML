function [iL1, vC1, Vout_end, iL_min, iL_max, Vout_min, Vout_max, iL_avg, Vout_avg, dcm, t_idle, ...
    iL_mid_on, Vout_mid_on] = sw_boost_period(iL, vC, d, R, Vin, L, C, Ts, rL, rC, Ron, VF, nsub, sync, ...
    pwm_center)
%SW_BOOST_PERIOD  Scalar, code-generation version of plant_switched_step.
%   [iL1, vC1, Vout_end, iL_min, iL_max, Vout_min, Vout_max, iL_avg, ...
%    Vout_avg, dcm, t_idle, iL_mid_on, Vout_mid_on] = sw_boost_period(iL, ...
%        vC, d, R, Vin, L, C, Ts, rL, rC, Ron, VF, nsub, sync, pwm_center)
%   Same circuit, topologies and exact solution as plant_switched_step (see
%   its help for the state equations and the PWM conventions); all
%   arguments are scalar doubles, sync is 0 (diode) or 1 (synchronous,
%   = P.sync), pwm_center is 0 (trailing edge, default when omitted) or 1
%   (centre aligned: ON [0, d*Ts/2), OFF, ON [Ts - d*Ts/2, Ts), = P.pwm
%   'center'), and dcm is returned as 0/1.  iL_mid_on / Vout_mid_on are the
%   values at the centre of the ON pulse: t = d*Ts/2 (trailing) or t = 0
%   (centre, i.e. the start sample).  The sample points are propagated one
%   by one with the precomputed step matrix, so the code has no arrays,
%   structs or dynamic sizing and can be pasted into a Simulink MATLAB
%   Function block executed every Ts (unit delays on iL1 and vC1 close the
%   loop).  Results equal plant_switched_step to rounding (~1e-13, see
%   tests/test_switched_plant and tests/test_centre_pwm).
%#codegen
if nargin < 15
    pwm_center = 0;
end
nsub = max(1, round(nsub));
d = min(max(d, 0), 1);
if iL < 0 && sync == 0
    iL = 0;
end
kR = 1 / (1 + rC / R);
gC = 1 / ((R + rC) * C);
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
center = (pwm_center ~= 0) && (n_on > 0);

iL_min = Inf; iL_max = -Inf; Vout_min = Inf; Vout_max = -Inf;
Si = 0; Sv = 0;
dcm = 0;
t_idle = 0;
Vout_end = kR * vC;
o = Vout_end;                     % (defined on every path for code generation)
iL_mid_on = iL;
Vout_mid_on = 0;
a = -(rL + Ron) / L;
n_b = 1;
t_half = 0;

% ON interval (trailing) or first ON half (centre)
if n_on > 0
    if center
        n_a = ceil(n_on / 2);
        n_b = max(n_on - n_a, 1);
        t_half = 0.5 * t_on;
        h = t_half / n_a;
        Vout_mid_on = kR * vC;
    else
        n_a = n_on;
        h = t_on / n_on;
        tm = 0.5 * t_on;
        if a == 0
            iL_mid_on = iL + Vin / L * tm;
        else
            iL_mid_on = iL * exp(a * tm) + Vin / L * expm1(a * tm) / a;
        end
        Vout_mid_on = kR * vC * exp(-gC * tm);
    end
    [iL, vC, o, Si, Sv, iL_min, iL_max, Vout_min, Vout_max] = sw_on(iL, vC, h, n_a, a, Vin, L, ...
        gC, kR, Si, Sv, iL_min, iL_max, Vout_min, Vout_max);
    Vout_end = o;
end

% OFF interval
if n_off > 0
    h = t_off / n_off;
    if sync ~= 0
        rOff = rL + Ron;
        vth = Vin;
        nev = 7;                      % no diode events
    else
        rOff = rL;
        vth = Vin - VF;
        nev = 0;
    end
    a11 = -(rOff + kR * rC) / L;
    a12 = -kR / L;
    a21 = kR / C;
    a22 = -gC;
    s = 0.5 * (a11 + a22);
    m11 = 0.5 * (a11 - a22);
    Dl = m11 * m11 + a12 * a21;
    iLeq = vth / (rOff + R);
    vCeq = vth / (1 + rOff / R);
    [H11, H12, H21, H22] = sw_e2(h, s, m11, a12, a21, Dl);
    eV = exp(-gC * h);

    idle = (nev < 7) && (iL <= 0) && (vth - kR * vC < 0);
    t_re = Inf;
    if idle
        dcm = 1;
        o = kR * vC;
        t_re = sw_reconduct(vC, kR, gC, vth);
    else
        o = kR * (vC + rC * iL);
    end
    iL_min = min(iL_min, iL); iL_max = max(iL_max, iL);
    Vout_min = min(Vout_min, o); Vout_max = max(Vout_max, o);
    if n_on == 0
        Vout_mid_on = o;
    end
    t = 0;
    k = 1;
    ongrid = true;
    while k <= n_off
        if k == n_off
            tn = t_off;
        else
            tn = k * h;
        end
        dt = tn - t;
        if ~idle
            if ongrid
                E11 = H11; E12 = H12; E21 = H21; E22 = H22;
            else
                [E11, E12, E21, E22] = sw_e2(dt, s, m11, a12, a21, Dl);
            end
            di = iL - iLeq;
            dv = vC - vCeq;
            i1 = iLeq + E11 * di + E12 * dv;
            v1 = vCeq + E21 * di + E22 * dv;
            if i1 < 0 && nev < 7
                % diode current reaches zero inside this step
                tz = sw_zero(dt, iL, i1, di, dv, s, m11, a12, a21, Dl, iLeq, a11);
                [E11, E12, E21, E22] = sw_e2(tz, s, m11, a12, a21, Dl);
                vz = vCeq + E21 * di + E22 * dv;
                oz = kR * vz;
                Si = Si + 0.5 * tz * iL;
                Sv = Sv + 0.5 * tz * (o + oz);
                iL_min = min(iL_min, 0);
                Vout_min = min(Vout_min, oz); Vout_max = max(Vout_max, oz);
                t = t + tz;
                iL = 0; vC = vz; o = oz;
                idle = true;
                dcm = 1;
                nev = nev + 1;
                ongrid = false;
                t_re = t + sw_reconduct(vC, kR, gC, vth);
                continue;
            end
            o1 = kR * (v1 + rC * i1);
        else
            if t_re < tn && nev < 7
                % diode starts conducting again inside this step
                dt = t_re - t;
                v1 = vC * exp(-gC * dt);
                o1 = kR * v1;
                Sv = Sv + 0.5 * dt * (o + o1);
                Vout_min = min(Vout_min, o1); Vout_max = max(Vout_max, o1);
                t_idle = t_idle + dt;
                t = t_re;
                vC = v1; o = o1;
                idle = false;
                nev = nev + 1;
                ongrid = false;
                continue;
            end
            if ongrid
                v1 = vC * eV;
            else
                v1 = vC * exp(-gC * dt);
            end
            i1 = 0;
            o1 = kR * v1;
            t_idle = t_idle + dt;
        end
        Si = Si + 0.5 * dt * (iL + i1);
        Sv = Sv + 0.5 * dt * (o + o1);
        iL_min = min(iL_min, i1); iL_max = max(iL_max, i1);
        Vout_min = min(Vout_min, o1); Vout_max = max(Vout_max, o1);
        iL = i1; vC = v1; o = o1;
        t = tn;
        k = k + 1;
        ongrid = true;
    end
    Vout_end = o;
end

% centre aligned: second ON half [Ts - t_on/2, Ts), uses this period's d
if center
    h = t_half / n_b;
    [iL, vC, o, Si, Sv, iL_min, iL_max, Vout_min, Vout_max] = sw_on(iL, vC, h, n_b, a, Vin, L, ...
        gC, kR, Si, Sv, iL_min, iL_max, Vout_min, Vout_max);
    Vout_end = o;
end

iL1 = iL;
vC1 = vC;
iL_avg = Si / Ts;
Vout_avg = Sv / Ts;
end

function [iL, vC, o, Si, Sv, iL_min, iL_max, Vout_min, Vout_max] = sw_on(iL, vC, h, n, a, Vin, L, ...
    gC, kR, Si, Sv, iL_min, iL_max, Vout_min, Vout_max)
% switch ON for n steps of h from (iL, vC); trapezoid sums and extremes
% (the start point is included, i.e. the ON side of the switching instant)
if a == 0
    eI = 1;
    gI = Vin / L * h;
else
    eI = exp(a * h);
    gI = Vin / L * expm1(a * h) / a;
end
eV = exp(-gC * h);
o = kR * vC;
iL_min = min(iL_min, iL); iL_max = max(iL_max, iL);
Vout_min = min(Vout_min, o); Vout_max = max(Vout_max, o);
for k = 1:n
    i1 = iL * eI + gI;
    v1 = vC * eV;
    o1 = kR * v1;
    Si = Si + 0.5 * h * (iL + i1);
    Sv = Sv + 0.5 * h * (o + o1);
    iL_min = min(iL_min, i1); iL_max = max(iL_max, i1);
    Vout_min = min(Vout_min, o1); Vout_max = max(Vout_max, o1);
    iL = i1; vC = v1; o = o1;
end
end

function [E11, E12, E21, E22] = sw_e2(t, s, m11, a12, a21, Dl)
% e^(A t) = e^(s t) [c I + S (A - s I)],  (A - s I)^2 = Dl I
e = exp(s * t);
if Dl < 0
    w = sqrt(-Dl);
    c = cos(w * t);
    S = sin(w * t) / w;
elseif Dl > 0
    mu = sqrt(Dl);
    c = cosh(mu * t);
    S = sinh(mu * t) / mu;
else
    c = 1;
    S = t;
end
E11 = e * (c + S * m11);
E12 = e * S * a12;
E21 = e * S * a21;
E22 = e * (c - S * m11);
end

function tr = sw_reconduct(vC, kR, gC, vth)
% time until Vout = kR*vC*exp(-gC t) falls to vth (idle -> conducting)
if vth <= 0
    tr = Inf;
elseif kR * vC <= vth
    tr = 0;
elseif gC > 0
    tr = log(kR * vC / vth) / gC;
else
    tr = Inf;
end
end

function tz = sw_zero(tb, fa, fb, di, dv, s, m11, a12, a21, Dl, iLeq, a11)
% zero of iL in [0, tb], iL(0) = fa >= 0 > fb = iL(tb); safeguarded Newton
ta = 0;
if fa <= 0
    tz = 0;
    return;
end
tol = 1e-13 * tb + 1e-300;
t = fa * tb / (fa - fb);
for it = 1:60
    [E11, E12, E21, E22] = sw_e2(t, s, m11, a12, a21, Dl);
    f = iLeq + E11 * di + E12 * dv;
    if f == 0
        break;
    elseif f > 0
        ta = t;
    else
        tb = t;
    end
    % diL/dt = a11*iL + a12*vC + b1 with vC - vCeq = E21*di + E22*dv and
    % a11*iLeq + a12*vCeq + b1 = 0
    fp = a11 * (f - iLeq) + a12 * (E21 * di + E22 * dv);
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
