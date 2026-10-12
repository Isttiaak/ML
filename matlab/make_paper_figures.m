function figs = make_paper_figures(res_sim, cfg)
%MAKE_PAPER_FIGURES  Paper figures from the corrected study (MATLAB graphics).
%   figs = make_paper_figures(res_sim)        study_config() defaults
%   figs = make_paper_figures(res_sim, cfg)   cfg from study_config (.fig used)
%
%   res_sim may be
%     - a struct with optional fields mopso (run_mopso_study result), rob
%       (run_robustness result) and traces (.<ctrl>.t/Vo/iL/Vo_min/Vo_max)
%     - a run_robustness result (time traces from rob.traces.nominal)
%     - a run_mopso_study result (fronts + HV; the nominal traces are then
%       simulated with run_robustness(scenario 'nominal'), cfg.rob settings)
%   Figures (each 3.5 in wide, Times New Roman 8 pt; files in cfg.fig.dir):
%     fig_vo        Vo(t) over the whole run
%     fig_il        iL(t) over the whole run (samples; with centre PWM the
%                   sample is the period average)
%     fig_loadstep  Vo zoom around the event (t_evt - 2 ms .. + 15 ms)
%     fig_ripple    steady-state switching ripple: per-period envelope
%                   Vo_min/Vo_max of the switched plant over the last 1 ms
%                   before the event (one panel per controller)
%     fig_pareto    merged Pareto fronts, first four objectives, ONE common
%                   normalisation over all controllers (non-penalised
%                   members; compromise points marked)
%     fig_hv        hypervolume history, mean +/- std over the seeds
%   Controllers in cfg.fig.exclude (default single_pi, unstable) are left out.
%   Export: exportgraphics at cfg.fig.dpi (600) when available, otherwise
%   print -r600 (Octave).  cfg.fig.dry_run = true only builds the plotted
%   data (figs(i).data) without creating figures.  Figures are invisible
%   (cfg.fig.visible 'off') and closed after saving.
%   figs: struct array (name, files, data, skipped).

if nargin < 2
    cfg = [];
end
cfg = study_config(cfg);
fo = cfg.fig;
if ischar(fo.formats)
    fo.formats = {fo.formats};
end
while ~isempty(fo.formats) && iscell(fo.formats{1})      % {{'png','pdf'}} from struct()
    fo.formats = [fo.formats{:}];
end
P = cfg.P;
[mres, rres, traces] = unpack(res_sim);
if isempty(traces) && ~isempty(mres)
    c2 = cfg;
    c2.rob.scenario_names = {'nominal'};
    c2.rob.keep_traces = {'nominal'};
    c2.rob.controllers = setdiff(mres.controllers, fo.exclude, 'stable');
    c2.rob.verbose = false;
    r = run_robustness(mres, c2);
    traces = getf(r.traces, 'nominal', struct());
    rres = r;
end
t_evt = P.t_step;
if ~isempty(rres)
    k = find(strcmp({rres.scenarios.name}, 'nominal'), 1);
    if ~isempty(k)
        t_evt = rres.scenarios(k).t_evt;
    end
end

if ~fo.dry_run && ~isempty(fo.dir) && ~exist(fo.dir, 'dir')
    mkdir(fo.dir);
end
figs = struct('name', {}, 'files', {}, 'data', {}, 'skipped', {});

% ---- time-domain figures
tc = {};
if ~isempty(traces)
    tc = setdiff(fieldnames(traces).', fo.exclude, 'stable');
    tc = order_ctrls(tc);
end
if isempty(tc)
    figs = add_skip(figs, {'fig_vo', 'fig_il', 'fig_loadstep', 'fig_ripple'}, 'no time traces');
else
    t_ms = traces.(tc{1}).t * 1e3;
    Vo = cols(traces, tc, 'Vo');
    iL = cols(traces, tc, 'iL');
    labels = cellfun(@drv_ctrl_label, tc, 'UniformOutput', false);

    d = struct('t_ms', t_ms, 'Y', Vo, 'labels', {labels}, 'ctrls', {tc}, 'Vref', P.Vref, 't_evt_ms', t_evt * 1e3);
    figs = draw(figs, 'fig_vo', d, fo, 2.2, @(f, d) plot_time(f, d, 'V_o (V)', [], d.Vref));
    d.Y = iL;
    figs = draw(figs, 'fig_il', d, fo, 2.2, @(f, d) plot_time(f, d, 'i_L (A)', [], []));
    sel = t_ms >= t_evt * 1e3 - 2 & t_ms <= t_evt * 1e3 + 15;
    d = struct('t_ms', t_ms(sel), 'Y', Vo(sel, :), 'labels', {labels}, 'ctrls', {tc}, ...
        'Vref', P.Vref, 't_evt_ms', t_evt * 1e3);
    figs = draw(figs, 'fig_loadstep', d, fo, 2.0, @(f, d) plot_time(f, d, 'V_o (V)', ...
        [d.t_ms(1) d.t_ms(end)], d.Vref));

    vmin = cols(traces, tc, 'Vo_min');
    vmax = cols(traces, tc, 'Vo_max');
    if isempty(vmin) || all(isnan(vmin(:)))
        figs = add_skip(figs, {'fig_ripple'}, 'no switched-plant envelope (Vo_min/Vo_max)');
    else
        sel = t_ms >= t_evt * 1e3 - 1 & t_ms < t_evt * 1e3;
        pp = max(vmax(sel, :), [], 1) - min(vmin(sel, :), [], 1);
        d = struct('t_ms', t_ms(sel), 'Vo_min', vmin(sel, :), 'Vo_max', vmax(sel, :), 'Vo', Vo(sel, :), ...
            'ripple_pp_V', pp, 'labels', {labels}, 'ctrls', {tc}, 'Vref', P.Vref);
        figs = draw(figs, 'fig_ripple', d, fo, 0.75 * numel(tc) + 0.5, @plot_ripple);
    end
end

% ---- optimisation figures
if isempty(mres)
    figs = add_skip(figs, {'fig_pareto', 'fig_hv'}, 'no run_mopso_study result');
else
    pc = order_ctrls(setdiff(mres.controllers, fo.exclude, 'stable'));
    d = pareto_data(mres, pc, cfg);
    if isempty(d.F)
        figs = add_skip(figs, {'fig_pareto'}, 'no non-penalised front members');
    else
        figs = draw(figs, 'fig_pareto', d, fo, 3.2, @plot_pareto);
    end
    d = hv_data(mres, pc);
    if isempty(d.ctrls)
        figs = add_skip(figs, {'fig_hv'}, 'no HV history (cfg.hv.history = false?)');
    else
        figs = draw(figs, 'fig_hv', d, fo, 2.0, @plot_hv);
    end
end
end

% ===========================================================================
function [mres, rres, traces] = unpack(r)
mres = [];
rres = [];
traces = [];
if isempty(r)
    return;
end
if isfield(r, 'summary') && isfield(r, 'ctrl')
    mres = r;
elseif isfield(r, 'scenarios') && isfield(r, 'traces')
    rres = r;
else
    mres = getf(r, 'mopso', []);
    rres = getf(r, 'rob', []);
    traces = getf(r, 'traces', []);
end
if isempty(traces) && ~isempty(rres) && isfield(rres.traces, 'nominal')
    traces = rres.traces.nominal;
end
end

function c = order_ctrls(c)
ref = {'fcsmpc', 'cascade_pi', 'cascade_pid', 'smc', 'single_pi'};
[~, i] = ismember(c, ref);
[~, o] = sort(i);
c = c(o);
end

function Y = cols(traces, tc, f)
Y = [];
for i = 1:numel(tc)
    v = getf(traces.(tc{i}), f, []);
    if isempty(v)
        Y = [];
        return;
    end
    Y(:, i) = v(:); %#ok<AGROW>
end
end

function figs = add_skip(figs, names, why)
for i = 1:numel(names)
    figs(end + 1) = struct('name', names{i}, 'files', {{}}, 'data', struct(), 'skipped', why); %#ok<AGROW>
end
end

function figs = draw(figs, name, d, fo, h_in, fn)
entry = struct('name', name, 'files', {{}}, 'data', d, 'skipped', '');
if fo.dry_run
    figs(end + 1) = entry;
    return;
end
if exist('OCTAVE_VERSION', 'builtin') ~= 0
    ws = warning('off', 'Octave:gnuplot-graphics');      % headless gnuplot notice
    restore = onCleanup(@() warning(ws));                   %#ok<NASGU>
end
fig = figure('Visible', fo.visible, 'Color', 'w', 'Units', 'inches', ...
    'Position', [1 1 fo.width_in h_in], 'PaperUnits', 'inches', ...
    'PaperPosition', [0 0 fo.width_in h_in], 'PaperSize', [fo.width_in h_in]);
try
    fn(fig, d);
    set(findall(fig, '-property', 'FontName'), 'FontName', fo.font);
    set(findall(fig, 'Type', 'axes'), 'FontSize', fo.font_size);
    files = {};
    for k = 1:numel(fo.formats)
        f = fullfile(fo.dir, [name '.' fo.formats{k}]);
        export_one(fig, f, fo.formats{k}, fo.dpi);
        files{end + 1} = f; %#ok<AGROW>
    end
    entry.files = files;
catch e
    entry.skipped = ['error: ' e.message];
    warning('make_paper_figures:draw', '%s: %s', name, e.message);
end
close(fig);
figs(end + 1) = entry;
end

function export_one(fig, f, fmt, dpi)
if exist('exportgraphics') > 0 %#ok<EXIST>
    if strcmpi(fmt, 'pdf') || strcmpi(fmt, 'eps')
        exportgraphics(fig, f, 'ContentType', 'vector');
    else
        exportgraphics(fig, f, 'Resolution', dpi);
    end
else
    switch lower(fmt)
        case 'pdf'
            print(fig, '-dpdf', f);
        case 'eps'
            print(fig, '-depsc', f);
        otherwise
            print(fig, ['-d' lower(fmt)], sprintf('-r%d', dpi), f);
    end
end
end

function [col, ls, lw] = style(ctrl)
switch ctrl
    case 'fcsmpc'
        col = [0 0 0]; ls = '-'; lw = 0.5;
    case 'cascade_pi'
        col = [0.839 0.153 0.157]; ls = '--'; lw = 1.0;
    case 'cascade_pid'
        col = [0.122 0.467 0.706]; ls = '-.'; lw = 1.0;
    case 'smc'
        col = [0.173 0.627 0.173]; ls = ':'; lw = 1.2;
    otherwise
        col = [1 0.498 0.055]; ls = '-'; lw = 1.0;
end
end

function plot_time(fig, d, ylab, xl, yref)
ax = axes('Parent', fig);
hold(ax, 'on');
h = zeros(1, numel(d.ctrls));
for i = 1:numel(d.ctrls)
    [col, ls, lw] = style(d.ctrls{i});
    h(i) = plot(ax, d.t_ms, d.Y(:, i), 'Color', col, 'LineStyle', ls, 'LineWidth', lw);
end
if ~isempty(yref)
    plot(ax, d.t_ms([1 end]), [yref yref], 'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'LineWidth', 0.5);
end
yl = ylim(ax);
plot(ax, [d.t_evt_ms d.t_evt_ms], yl, 'Color', [0.5 0.5 0.5], 'LineStyle', ':', 'LineWidth', 0.5);
ylim(ax, yl);
if isempty(xl)
    xl = [d.t_ms(1) d.t_ms(end)];
end
xlim(ax, xl);
xlabel(ax, 'Time (ms)');
ylabel(ax, ylab);
legend(ax, h, d.labels, 'Location', best_loc());
legend(ax, 'boxoff');
box(ax, 'on');
hold(ax, 'off');
end

function plot_ripple(fig, d)
n = numel(d.ctrls);
dt = d.t_ms(2) - d.t_ms(1);
for i = 1:n
    ax = subplot(n, 1, i, 'Parent', fig);
    hold(ax, 'on');
    [col, ~, ~] = style(d.ctrls{i});
    stairs(ax, d.t_ms, d.Vo_max(:, i), 'Color', col, 'LineWidth', 0.6);
    stairs(ax, d.t_ms, d.Vo_min(:, i), 'Color', col, 'LineWidth', 0.6);
    plot(ax, d.t_ms, d.Vo(:, i), 'k.', 'MarkerSize', 3);
    xlim(ax, [d.t_ms(1) d.t_ms(end) + dt]);
    lo = min(d.Vo_min(:, i));
    hi = max(d.Vo_max(:, i));
    pad = 0.25 * max(hi - lo, 1e-3);
    ylim(ax, [lo - pad, hi + 2 * pad]);
    set(ax, 'YTick', unique(round(linspace(lo, hi, 3) * 1e3) / 1e3));
    text(0.02, 0.97, sprintf('%s: %.3f V pp', d.labels{i}, d.ripple_pp_V(i)), 'Parent', ax, ...
        'Units', 'normalized', 'VerticalAlignment', 'top');
    if i == n
        xlabel(ax, 'Time (ms)');
    else
        set(ax, 'XTickLabel', {});
    end
    if i == ceil(n / 2)
        ylabel(ax, 'V_o (V)');
    end
    box(ax, 'on');
    hold(ax, 'off');
end
end

function d = pareto_data(mres, pc, cfg)
d = struct('F', [], 'ctrl_idx', [], 'comp', [], 'ctrls', {{}}, 'labels', {{}}, ...
    'obj_names', {{}}, 'ideal', [], 'nadir', [], 'pairs', [1 2; 1 3; 1 4; 3 4]);
Fall = [];
idx = [];
comp = [];
kept = {};
for i = 1:numel(pc)
    R = mres.ctrl.(pc{i});
    mg = R.merged;
    F = mg.F(~mg.penalised, 1:4);
    if isempty(F)
        continue;
    end
    kept{end + 1} = pc{i}; %#ok<AGROW>
    Fall = [Fall; F]; %#ok<AGROW>
    idx = [idx; numel(kept) * ones(size(F, 1), 1)]; %#ok<AGROW>
    comp = [comp; mg.f(1:4)]; %#ok<AGROW>
    d.obj_names = R.obj_names(1:4);
end
if isempty(Fall)
    return;
end
lo = min(Fall, [], 1);
hi = max(Fall, [], 1);
sp = hi - lo;
sp(~(sp > 0)) = 1;
d.F = (Fall - lo) ./ sp;
d.comp = (comp - lo) ./ sp;
d.ctrl_idx = idx;
d.ctrls = kept;
d.labels = cellfun(@drv_ctrl_label, kept, 'UniformOutput', false);
d.ideal = lo;
d.nadir = hi;
d.objective_set = cfg.objective_set;
end

function plot_pareto(fig, d)
mk = {'o', 's', 'd', '^', 'v'};
names = cellfun(@obj_label, d.obj_names, 'UniformOutput', false);
for p = 1:size(d.pairs, 1)
    ax = subplot(2, 2, p, 'Parent', fig);
    hold(ax, 'on');
    a = d.pairs(p, 1);
    b = d.pairs(p, 2);
    h = zeros(1, numel(d.ctrls));
    for i = 1:numel(d.ctrls)
        [col, ~, ~] = style(d.ctrls{i});
        r = d.ctrl_idx == i;
        h(i) = plot(ax, d.F(r, a), d.F(r, b), mk{i}, 'Color', col, 'MarkerSize', 2.5, ...
            'LineStyle', 'none');
        plot(ax, d.comp(i, a), d.comp(i, b), mk{i}, 'Color', col, 'MarkerFaceColor', col, ...
            'MarkerSize', 5, 'LineStyle', 'none');
    end
    xlabel(ax, names{a});
    ylabel(ax, names{b});
    xlim(ax, [-0.05 1.05]);
    ylim(ax, [-0.05 1.05]);
    box(ax, 'on');
    if p == 1
        legend(ax, h, d.labels, 'Location', 'northeast');
        legend(ax, 'boxoff');
    end
    hold(ax, 'off');
end
end

function s = obj_label(name)
switch name
    case {'startup_settling_ms', 'settling_ms'}
        s = 'Settling';
    case {'startup_overshoot_pct', 'overshoot_pct'}
        s = 'Overshoot';
    case 'event_peak_dev_V'
        s = 'Peak deviation';
    case 'recovery_ms'
        s = 'Recovery';
    case 'ss_error_V'
        s = 'SS error';
    case 'load_reg_V'
        s = 'Load reg.';
    otherwise
        s = strrep(name, '_', ' ');
end
s = [s ' (norm.)'];
end

function d = hv_data(mres, pc)
d = struct('ctrls', {{}}, 'labels', {{}}, 'evals', {{}}, 'mean', {{}}, 'std', {{}}, 'n_seeds', []);
for i = 1:numel(pc)
    R = mres.ctrl.(pc{i});
    H = R.hv_hist;
    if isempty(H) || all(isnan(H(:)))
        continue;
    end
    d.ctrls{end + 1} = pc{i};
    d.labels{end + 1} = drv_ctrl_label(pc{i});
    d.evals{end + 1} = R.hv_hist_evals;
    d.mean{end + 1} = mean(H, 2);
    if size(H, 2) > 1
        d.std{end + 1} = std(H, 0, 2);
    else
        d.std{end + 1} = zeros(size(H, 1), 1);
    end
    d.n_seeds(end + 1) = size(H, 2);
end
end

function plot_hv(fig, d)
ax = axes('Parent', fig);
hold(ax, 'on');
h = zeros(1, numel(d.ctrls));
for i = 1:numel(d.ctrls)
    [col, ls, lw] = style(d.ctrls{i});
    h(i) = plot(ax, d.evals{i}, d.mean{i}, 'Color', col, 'LineStyle', ls, 'LineWidth', max(lw, 0.8));
    plot(ax, d.evals{i}, d.mean{i} + d.std{i}, 'Color', col, 'LineStyle', '-', 'LineWidth', 0.3);
    plot(ax, d.evals{i}, max(d.mean{i} - d.std{i}, 0), 'Color', col, 'LineStyle', '-', 'LineWidth', 0.3);
end
xlabel(ax, 'Objective evaluations per seed');
ylabel(ax, 'Hypervolume (normalised)');
legend(ax, h, d.labels, 'Location', 'southeast');
legend(ax, 'boxoff');
box(ax, 'on');
hold(ax, 'off');
end

function loc = best_loc()
% Octave 8 has no 'best' legend location
if exist('OCTAVE_VERSION', 'builtin') ~= 0
    loc = 'northeast';
else
    loc = 'best';
end
end

function v = getf(s, name, default)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
