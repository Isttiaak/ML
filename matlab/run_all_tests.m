function results = run_all_tests(sel, o)
%RUN_ALL_TESTS  Run the test suite in tests/ and print a PASS/FAIL/SKIP summary.
%   results = run_all_tests()                every tests/test_*.m
%   results = run_all_tests({'test_drivers'}) a subset (names without .m)
%   results = run_all_tests([], o)           options o (all optional):
%     quiet         false; true captures each test's output (evalc) and
%                   prints it only when the test fails
%     stop_on_fail  false
%     no_error      false; by default an error is raised after the summary
%                   when a test failed (non-zero exit status for CI)
%   Order: test_core_equivalence, test_metrics_event, test_mopso,
%   test_switched_plant, test_centre_pwm, test_simulink_blocks, the others
%   alphabetically, test_drivers last.  Every test is called without
%   arguments; a test passes when it returns without an error.
%   SKIP (not counted as failure, reason printed):
%     - the test source uses Simulink API calls (load_system, new_system,
%       open_system, parsim, Simulink.SimulationInput) and Simulink is not
%       available (always the case in Octave)
%     - the test stops on an undefined function that is not on the path
%       (missing dependency, e.g. a file another work package has not added)
%   results: struct array name, status ('PASS'|'FAIL'|'SKIP'), time_s, message.

if nargin < 1
    sel = [];
end
if nargin < 2 || isempty(o)
    o = struct();
end
quiet = getf(o, 'quiet', false);
stop_on_fail = getf(o, 'stop_on_fail', false);
no_error = getf(o, 'no_error', false);

here = fileparts(mfilename('fullpath'));
tdir = fullfile(here, 'tests');
addpath(here);
addpath(tdir);
d = dir(fullfile(tdir, 'test_*.m'));
names = sort(cellfun(@(f) f(1:end - 2), {d.name}, 'UniformOutput', false));
first = {'test_core_equivalence', 'test_metrics_event', 'test_mopso', 'test_switched_plant', ...
    'test_centre_pwm', 'test_simulink_blocks'};
last = {'test_drivers'};
mid = setdiff(names, [first last]);
ordered = [first(ismember(first, names)), mid(:).', last(ismember(last, names))];
if ~isempty(sel)
    if ischar(sel)
        sel = {sel};
    end
    sel = regexprep(sel, '\.m$', '');
    missing = setdiff(sel, names);
    if ~isempty(missing)
        error('run_all_tests:unknown', 'unknown test(s): %s', strjoin(missing, ', '));
    end
    ordered = ordered(ismember(ordered, sel));
end
has_sl = simulink_available();

results = struct('name', {}, 'status', {}, 'time_s', {}, 'message', {});
t_all = tic;
for i = 1:numel(ordered)
    name = ordered{i};
    r = struct('name', name, 'status', 'PASS', 'time_s', 0, 'message', '');
    src = fileread(fullfile(tdir, [name '.m']));
    if ~has_sl && ~isempty(regexp(src, ['(load_system|new_system|open_system|parsim|' ...
            'Simulink\.SimulationInput)\s*\('], 'once'))
        r.status = 'SKIP';
        r.message = 'needs Simulink (not available here)';
        results(end + 1) = r; %#ok<AGROW>
        fprintf('==== %s: SKIP (%s)\n', name, r.message);
        continue;
    end
    fprintf('==== %s\n', name);
    t0 = tic;
    out = '';
    try
        if quiet
            out = evalc('feval(name);');
        else
            feval(name);
        end
    catch e
        miss = missing_function(e);
        if ~isempty(miss)
            r.status = 'SKIP';
            r.message = sprintf('missing dependency: %s', miss);
        else
            r.status = 'FAIL';
            r.message = e.message;
            if quiet && ~isempty(out)
                fprintf('%s', out);
            end
        end
    end
    r.time_s = toc(t0);
    close_figures();
    results(end + 1) = r; %#ok<AGROW>
    fprintf('==== %s: %s (%.1f s)%s\n', name, r.status, r.time_s, msg_txt(r.message));
    if stop_on_fail && strcmp(r.status, 'FAIL')
        break;
    end
end

fprintf('\n%-26s %-5s %8s  %s\n', 'test', 'state', 'time[s]', 'message');
for i = 1:numel(results)
    r = results(i);
    m = r.message;
    if numel(m) > 70
        m = [m(1:67) '...'];
    end
    fprintf('%-26s %-5s %8.1f  %s\n', r.name, r.status, r.time_s, m);
end
st = {results.status};
fprintf('%d passed, %d failed, %d skipped (%.1f s)\n', sum(strcmp(st, 'PASS')), ...
    sum(strcmp(st, 'FAIL')), sum(strcmp(st, 'SKIP')), toc(t_all));
if any(strcmp(st, 'FAIL')) && ~no_error
    error('run_all_tests:failed', '%d test(s) failed', sum(strcmp(st, 'FAIL')));
end
end

% ---------------------------------------------------------------------------
function tf = simulink_available()
tf = false;
if exist('OCTAVE_VERSION', 'builtin') ~= 0
    return;
end
try
    tf = exist('load_system') > 0 && license('test', 'Simulink'); %#ok<EXIST>
catch
    tf = false;
end
end

function name = missing_function(e)
% name of an undefined function that is not on the path, '' otherwise
name = '';
id = e.identifier;
if ~any(strcmp(id, {'Octave:undefined-function', 'MATLAB:UndefinedFunction', ...
        'MATLAB:undefinedVarOrFunction', 'MATLAB:UndefinedFunctionOrVariable'}))
    return;
end
tok = regexp(e.message, '''([A-Za-z]\w*)''', 'tokens', 'once');
if isempty(tok)
    return;
end
if exist(tok{1}) == 0 %#ok<EXIST>
    name = tok{1};
end
end

function close_figures()
try
    close('all', 'hidden');
catch
    close('all');
end
end

function s = msg_txt(m)
if isempty(m)
    s = '';
else
    s = [' - ' m];
end
end

function v = getf(s, name, default)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
