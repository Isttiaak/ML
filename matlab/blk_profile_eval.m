function v = blk_profile_eval(t, PV)
%BLK_PROFILE_EVAL  Scalar load / input-voltage profile for a Simulink block.
%   v = blk_profile_eval(t, PV)
%     t  : time [s], computed by the caller as k*Ts (like Python / sim_boost)
%     PV : 1x7 numeric profile vector [kind p1 p2 p3 p4 p5 p6] built by
%          make_profile_vec:
%       kind 0 const       : v = p1
%       kind 1 step        : p1 t_chg, p2 value before, p3 value for t >= t_chg
%       kind 2 ramp        : p1 t0, p2 t1, p3 v0, p4 v_end
%                            v0 for t < t0, v0 + (t-t0)/(t1-t0)*(v_end-v0) for
%                            t0 <= t <= t1, v_end for t > t1
%       kind 3 double_step : p1 t1, p2 t2, p3 v0, p4 v_mid, p5 v_end
%                            v_mid for t1 <= t < t2, v_end for t >= t2
%   Same comparisons and floating-point order as load_profile/vin_profile,
%   so the result is bit-identical to them on the grid t = (0:N-1)*Ts.
%   Unknown kinds return NaN.
%#codegen
kind = PV(1);
if kind == 0
    v = PV(2);
elseif kind == 1
    if t >= PV(2)
        v = PV(4);
    else
        v = PV(3);
    end
elseif kind == 2
    t0 = PV(2);
    t1 = PV(3);
    v0 = PV(4);
    v_end = PV(5);
    if t > t1
        v = v_end;
    elseif t >= t0
        v = v0 + (t - t0) / (t1 - t0) * (v_end - v0);
    else
        v = v0;
    end
elseif kind == 3
    if t >= PV(3)
        v = PV(6);
    elseif t >= PV(2)
        v = PV(5);
    else
        v = PV(4);
    end
else
    v = NaN;
end
end
