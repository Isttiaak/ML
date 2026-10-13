%RUN_ME  Start here: open and run the Simulink boost-converter models.
%   In MATLAB: cd to this folder, then type   run_me
%   It
%     1. puts this folder on the path (the MATLAB Function blocks call blk_*.m)
%     2. uses boost_avg.slx / boost_cycle.slx from this folder, or builds them
%        with build_boost_models if they are missing (older MATLAB than
%        R2023b: use the copies in models_R2021b, see the README "Start here")
%     3. runs boost_avg for the controller below and checks it against the
%        script simulation sim_boost (they must agree to ~1e-12)
%     4. runs boost_cycle (exact switching plant, centre-aligned PWM) and
%        shows the real switching ripple
%     5. runs boost_simscape (Simscape circuit) if Simscape is installed
%     6. optionally runs the full validation check_simulink_models /
%        check_simscape_model
%   Change the settings block to try another controller, gains or scenario.
%   The README explains every option.

here = fileparts(mfilename('fullpath'));
cd(here);
addpath(here);
addpath(fullfile(here, 'tests'));

% ------------------------------------------------------------ settings
ctrl  = 'cascade_pi';      % 'fcsmpc' | 'cascade_pi' | 'cascade_pid' | 'smc' | 'single_pi'
gains = [];                % [] = notebook baseline gains; e.g. [0.0530583 1.0]
opts  = struct();          % [] / struct() = notebook (legacy) behaviour.  Corrected set-up:
% opts = struct('ff_mode','measured','delay',1);   % + 'KPV',..,'KIV',.. for tuned outer gains
run_simscape   = true;     % also run boost_simscape.slx when Simscape is installed
run_full_check = false;    % true: also run check_simulink_models / check_simscape_model
% ---------------------------------------------------------------------

P = boost_params();

% 1. models
need = {};
if ~isfile(fullfile(here, 'boost_avg.slx'))
    need{end + 1} = 'avg';
end
if ~isfile(fullfile(here, 'boost_cycle.slx'))
    need{end + 1} = 'cycle';
end
if numel(need) == 2
    build_boost_models('both', here);
elseif numel(need) == 1
    build_boost_models(need{1}, here);
end

% 2. averaged model (reproduces the Python notebook)
load_system('boost_avg');
setup_boost_workspace('boost_avg', ctrl, gains, P, opts);
so = sim('boost_avg');
y = so.ylog;                                   % [Vo iL d IL_ref R Vin]
t = (0:size(y, 1) - 1)' * P.Ts;
ref = sim_boost(ctrl, gains, P, opts);
fprintf('boost_avg (%s): max |Vo - sim_boost| = %.3g V, max |iL - sim_boost| = %.3g A\n', ...
    ctrl, max(abs(y(:, 1) - ref.Vo)), max(abs(y(:, 2) - ref.iL)));
if strcmp(ctrl, 'single_pi')
    disp('note: single_pi is unstable on this plant (see the review); expect oscillation');
end
m = metrics_legacy(y(:, 1), P);
fprintf('notebook metrics: settling %.2f ms, overshoot %.4f %%, SS error %.4f V, load reg %.4f V\n', ...
    m.settling_ms, m.overshoot_pct, m.ss_error_V, m.load_reg_V);

% 3. switching model (centre-aligned PWM, period min/max envelope)
ocyc = opts;
ocyc.plant = 'switched';
ocyc.pwm = 'center';
load_system('boost_cycle');
setup_boost_workspace('boost_cycle', ctrl, gains, P, ocyc);
sc = sim('boost_cycle');
yc = sc.ylog;
env = squeeze(sc.yenv);                        % [Vo_min Vo_max iL_min iL_max iL_avg Vo_avg dcm]
if size(env, 1) ~= size(yc, 1)
    env = env.';
end
k = round(0.045 / P.Ts):round(0.050 / P.Ts) - 1;
% switching ripple = max - min inside each 50 us period, averaged over 45-50 ms
% (max - min over the whole window would add the slow drift of Vo)
fprintf('boost_cycle: switching ripple 45-50 ms: Vo %.3f V pk-pk, iL %.3f A pk-pk (per period)\n', ...
    mean(env(k, 2) - env(k, 1)), mean(env(k, 4) - env(k, 3)));

% 4. plots
figure('Name', 'boost converter');
subplot(2, 1, 1);
plot(t * 1e3, y(:, 1), t * 1e3, yc(:, 1), '--');
hold on;
plot(t * 1e3, env(:, 1), ':', t * 1e3, env(:, 2), ':');
yline(P.Vref, 'k:');
ylabel('V_o (V)');
legend('boost\_avg', 'boost\_cycle (sample)', 'period min', 'period max', 'V_{ref}', 'Location', 'best');
title(sprintf('%s', strrep(ctrl, '_', '\_')));
subplot(2, 1, 2);
plot(t * 1e3, y(:, 2), t * 1e3, yc(:, 2), '--');
ylabel('i_L (A)');
xlabel('time (ms)');

% 5. Simscape circuit (only if Simscape is installed): same controller,
%    real inductor / switch / diode / capacitor, solver step 0.1 us
if run_simscape && isfile(fullfile(here, 'boost_simscape.slx')) && ~isempty(ver('simscape'))
    osc = opts;
    if ~isfield(osc, 'delay') || isempty(osc.delay)
        osc.delay = 1;                         % the circuit model has a fixed 1-sample delay
    end
    load_system('boost_simscape');
    setup_boost_simscape('boost_simscape', ctrl, gains, P, osc);
    ss = sim('boost_simscape');                % about 10 s for 100 ms
    ys = ss.ylog;                              % [vo iL d IL_ref R Vin] at Ts
    fine = squeeze(ss.yfine);                  % [iL vo] every 0.1 us
    if size(fine, 2) ~= 2
        fine = fine.';
    end
    tf = (0:size(fine, 1) - 1)' * P.Ts / 500;
    nper = round(P.Ts / (tf(2) - tf(1)));      % fine samples per 50 us period
    rip = zeros(numel(k), 1);                  % per-period max - min of the 0.1 us waveform
    for j = 1:numel(k)
        seg = fine((k(j) - 1) * nper + (1:nper + 1), 2);
        rip(j) = max(seg) - min(seg);
    end
    fprintf('boost_simscape: Vo at 45-50 ms %.3f V mean, switching ripple %.3f V pk-pk (per period)\n', ...
        mean(ys(k, 1)), mean(rip));
    figure('Name', 'boost_simscape');
    plot(tf * 1e3, fine(:, 2), t * 1e3, ys(:, 1), '.');
    xlabel('time (ms)');
    ylabel('V_o (V)');
    legend('circuit (0.1 \mus)', 'sampled at k T_s', 'Location', 'best');
end

% 6. full validation (Simulink vs scripts vs Python traces)
if run_full_check
    R = check_simulink_models(struct('folder', here));
    if isfile(fullfile(here, 'boost_simscape.slx')) && ~isempty(ver('simscape'))
        RS = check_simscape_model(struct('folder', here));
    end
end
