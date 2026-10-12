function [R, Vin, k_next] = blk_scenario(k, LP, VP, Ts)
%BLK_SCENARIO  Scenario source of the Simulink models (load R and input Vin).
%   [R, Vin, k_next] = blk_scenario(k, LP, VP, Ts)
%     k      sample counter 0, 1, 2, ... (Unit Delay 'k', initial condition 0,
%            fed back from k_next)
%     LP, VP 1x7 profile vectors (make_profile_vec(spec,'load'/'vin',P)),
%            block Parameters
%     Ts     sample time [s]
%   t = k*Ts is formed exactly like t = (0:N-1)'*Ts in sim_boost (and like
%   Python), so the t >= t_chg comparisons switch at the same sample; using
%   the Simulink clock instead could move a step by one sample.
%#codegen
t = k * Ts;
R = blk_profile_eval(t, LP);
Vin = blk_profile_eval(t, VP);
k_next = k + 1;
end
