function files = sw_crosscheck_export(csv_dir)
%SW_CROSSCHECK_EXPORT  Write plant_switched_step runs for switched_crosscheck.py.
%   files = sw_crosscheck_export(csv_dir) runs five open-loop cases of 200
%   periods with fixed duty / load / Vin sequences and writes
%     <csv_dir>/sw_cases.csv          one row of parameters per case
%     <csv_dir>/sw_case_<name>.csv    per period: inputs and all outputs
%   The Python script re-simulates the same circuit independently (fine
%   RK4) from the same initial state and sequences and compares.
if ~exist(csv_dir, 'dir')
    mkdir(csv_dir);
end
P0 = boost_params();
n = 200;
k = (0:n - 1).';
lcg = sw_lcg(n, 12345);

% name, rL, rC, Ron, VF, sync, nsub, iL0, vC0, d, R, Vin
cs = {};
d = min(max(0.5 + 0.3 * sin(2 * pi * k / 50), 0.05), 0.9);
cs(end + 1, :) = {'ideal_startup', 0, 0, 0, 0, 0, 200, 0, 24, d, ...
    20 + (k >= 100) * (10 - 20), 24 * ones(n, 1)};
cs(end + 1, :) = {'parasitic_mix', 0.05, 0.02, 0.03, 0.8, 0, 200, 5, 45, ...
    0.35 + 0.3 * lcg, 20 + (k >= 120) * (8 - 20), 24 + (k >= 60) * (20 - 24)};
cs(end + 1, :) = {'dcm_light', 0.02, 0.01, 0.01, 0.5, 0, 200, 0, 50, ...
    0.15 + 0.25 * mod(k, 7) / 6, 200 + (k >= 100) * (500 - 200), 24 * ones(n, 1)};
cs(end + 1, :) = {'sync_startup', 0.03, 0.01, 0.02, 0, 1, 200, 0, 24, ...
    0.5 * ones(n, 1), 20 * ones(n, 1), 24 * ones(n, 1)};
d = lcg;
d(1:3) = 0; d(4) = 1; d(5) = 0.999; d(6) = 0.001;
d(mod(k, 17) == 9) = 0;
d(mod(k, 23) == 11) = 1;
cs(end + 1, :) = {'edges', 0, 0, 0, 0, 0, 7, 0, 24.05, d, ...
    1 + (k >= 20) * (20 - 1), 24 * ones(n, 1)};

fid = fopen(fullfile(csv_dir, 'sw_cases.csv'), 'w');
fprintf(fid, 'name,L,C,Ts,rL,rC,Ron,VF,sync,nsub,iL0,vC0,nper\n');
files = {fullfile(csv_dir, 'sw_cases.csv')};
for c = 1:size(cs, 1)
    P = P0;
    P.rL = cs{c, 2}; P.rC = cs{c, 3}; P.Ron = cs{c, 4}; P.VF = cs{c, 5};
    P.sync = cs{c, 6} ~= 0;
    nsub = cs{c, 7};
    iL = cs{c, 8}; vC = cs{c, 9};
    dd = cs{c, 10}; RR = cs{c, 11}; VV = cs{c, 12};
    fprintf(fid, '%s,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%d,%d,%.17g,%.17g,%d\n', ...
        cs{c, 1}, P.L, P.C, P.Ts, P.rL, P.rC, P.Ron, P.VF, cs{c, 6}, nsub, iL, vC, n);
    M = zeros(n, 15);
    for j = 1:n
        [iL, vC, info] = plant_switched_step(iL, vC, dd(j), RR(j), VV(j), P, nsub);
        M(j, :) = [j - 1, dd(j), RR(j), VV(j), iL, vC, info.Vout_end, info.iL_avg, ...
            info.Vout_avg, info.iL_min, info.iL_max, info.Vout_min, info.Vout_max, ...
            double(info.dcm), info.t_idle];
    end
    fn = fullfile(csv_dir, ['sw_case_' cs{c, 1} '.csv']);
    f2 = fopen(fn, 'w');
    fprintf(f2, 'k,d,R,Vin,iL1,vC1,Vout_end,iL_avg,Vout_avg,iL_min,iL_max,Vout_min,Vout_max,dcm,t_idle\n');
    fprintf(f2, [repmat('%.17g,', 1, 14) '%.17g\n'], M.');
    fclose(f2);
    files{end + 1} = fn; %#ok<AGROW>
end
fclose(fid);
end

function u = sw_lcg(n, seed)
% deterministic uniform [0,1) sequence (Park-Miller), same on any platform
u = zeros(n, 1);
x = seed;
for j = 1:n
    x = mod(16807 * x, 2147483647);
    u(j) = x / 2147483647;
end
end
