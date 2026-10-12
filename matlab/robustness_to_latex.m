function str = robustness_to_latex(res, file, o)
%ROBUSTNESS_TO_LATEX  IEEE two-column (table*) LaTeX table of run_robustness.
%   str = robustness_to_latex(res)              returns the LaTeX string
%   str = robustness_to_latex(res, file)        also writes it to file
%   str = robustness_to_latex(res, file, o)     options o (all optional):
%     caption  plain text (escaped); default describes the battery set-up
%     label    'tab:robustness'
%     metrics  subset of res.metrics (default all)
%   Every text cell (scenario labels, controller names, caption) is passed
%   through drv_latex_escape, so %, _, &, # etc. can never break the table
%   (the notebook printed 'Load -50% (base)' unescaped, which commented out
%   the rest of the row).  Precision: ms and % with 2 decimals, V with 3.
%   NaN -> '--' (explained in the table note).  Monte-Carlo rows are
%   'mean $\pm$ std'; a '$^{*}$' marks entries where some runs gave NaN
%   (statistics over the remaining runs); '$^{\dagger}$' after a controller
%   name marks a run that diverged (res.diverged).
if nargin < 2
    file = '';
end
if nargin < 3 || isempty(o)
    o = struct();
end
metrics = getf(o, 'metrics', res.metrics);
[~, mi] = ismember(metrics, res.metrics);
if any(mi == 0)
    error('robustness_to_latex:metrics', 'unknown metric(s) requested');
end
hdr_map = struct( ...
    'startup_settling_ms', '$t_{s}$ (ms)', ...
    'startup_overshoot_pct', '$M_{p}$ (\%)', ...
    'pre_offset_V', '$|\bar{e}|_{\mathrm{pre}}$ (V)', ...
    'pre_ripple_pp_V', '$\Delta V_{\mathrm{pp}}$ (V)', ...
    'event_peak_dev_V', '$\Delta V_{\mathrm{pk}}$ (V)', ...
    'recovery_ms', '$t_{\mathrm{rec}}$ (ms)', ...
    'post_offset_V', '$|\bar{e}|_{\mathrm{post}}$ (V)');
nl = char(10);
caption = getf(o, 'caption', default_caption(res));
label = getf(o, 'label', 'tab:robustness');

L = {};
L{end + 1} = '\begin{table*}[!t]';
L{end + 1} = '\renewcommand{\arraystretch}{1.15}';
L{end + 1} = ['\caption{' drv_latex_escape(caption) '}'];
L{end + 1} = ['\label{' drv_latex_escape(label) '}'];
L{end + 1} = '\centering';
L{end + 1} = '\footnotesize';
L{end + 1} = ['\begin{tabular}{l l ' repmat('r ', 1, numel(mi)) '}'];
L{end + 1} = '\hline\hline';
h = cell(1, numel(mi));
for j = 1:numel(mi)
    key = res.metrics{mi(j)};
    if isfield(hdr_map, key)
        h{j} = hdr_map.(key);
    else
        h{j} = drv_latex_escape(key);
    end
end
L{end + 1} = ['Scenario & Controller & ' strjoin(h, ' & ') ' \\'];
L{end + 1} = '\hline';
any_nan = false;
any_star = false;
any_div = false;
div = false(numel(res.scenarios), numel(res.controllers));
if isfield(res, 'diverged') && isequal(size(res.diverged), size(div))
    div = logical(res.diverged);
end
nS = numel(res.scenarios);
for s = 1:nS
    sc = res.scenarios(s);
    for c = 1:numel(res.controllers)
        if c == 1
            lab = sc.label;
            if sc.mc
                lab = sprintf('%s, n = %d', lab, sc.n_runs);
            end
            first = drv_latex_escape(lab);
        else
            first = '';
        end
        cells = cell(1, numel(mi));
        for j = 1:numel(mi)
            k = mi(j);
            mu = res.mean(s, c, k);
            sd = res.std(s, c, k);
            nn = res.n_nan(s, c, k);
            if isnan(mu)
                cells{j} = '--';
                any_nan = true;
                continue;
            end
            fmt = unit_fmt(res.metric_units{k});
            if sc.mc
                if isnan(sd)
                    sd = 0;
                end
                cells{j} = sprintf([fmt ' $\\pm$ ' fmt], mu, sd);
            else
                cells{j} = sprintf(fmt, mu);
            end
            if nn > 0
                cells{j} = [cells{j} '$^{*}$'];
                any_star = true;
            end
        end
        cname = drv_latex_escape(drv_ctrl_label(res.controllers{c}));
        if div(s, c)
            cname = [cname '$^{\dagger}$'];
            any_div = true;
        end
        L{end + 1} = sprintf('%s & %s & %s \\\\', first, cname, strjoin(cells, ' & ')); %#ok<AGROW>
    end
    if s < nS
        L{end + 1} = '\hline'; %#ok<AGROW>
    end
end
L{end + 1} = '\hline\hline';
L{end + 1} = '\end{tabular}';
note = {};
if any_nan
    note{end + 1} = ['--: not settled before the event ($t_{s}$), not back in the band ' ...
        'at the end of the run ($t_{\mathrm{rec}}$), or not applicable (ripple needs the ' ...
        'switched plant).'];
end
if any_star
    note{end + 1} = '$^{*}$: some Monte-Carlo runs gave --; statistics over the others.';
end
if any_div
    note{end + 1} = '$^{\dagger}$: unstable, $|V_o|$ exceeded $2V_{\mathrm{ref}}$.';
end
if ~isempty(note)
    L{end + 1} = ['\par\smallskip\parbox{\textwidth}{\scriptsize ' strjoin(note, ' ') '}'];
end
L{end + 1} = '\end{table*}';
str = strjoin(L, nl);
str = [str nl];

if ~isempty(file)
    fid = fopen(file, 'w');
    if fid < 0
        error('robustness_to_latex:file', 'cannot open %s', file);
    end
    fprintf(fid, '%s', str);
    fclose(fid);
end
end

function fmt = unit_fmt(u)
switch u
    case 'V'
        fmt = '%.3f';
    otherwise          % ms and %
        fmt = '%.2f';
end
end

function c = default_caption(res)
pl = getf(res.opts, 'plant', 'avg');
if strcmp(pl, 'switched')
    pw = getf(res.opts, 'pwm', 'trailing');
    pl = sprintf('switched plant (%s-aligned PWM)', pw);
else
    pl = 'averaged plant';
end
c = sprintf(['Robustness of the tuned controllers: %s, %s feed-forward, %d-sample ' ...
    'computation delay; metrics after each scenario event, band +/-%g%% of Vref'], ...
    pl, getf(res.opts, 'ff_mode', 'oracle'), getf(res.opts, 'delay', 0), getf(res, 'band_pct', 2));
end

function v = getf(s, name, default)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
