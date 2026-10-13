function ci_build()
%CI_BUILD  Build and validate the Simulink models on real MATLAB (used by CI).
%   ci_build()   run from any folder; everything happens next to this file.
%   Used by .github/workflows/build-simulink-models.yml (MATLAB + Simulink on
%   a GitHub runner), but works the same on a desktop MATLAB.  Steps:
%     1. build_boost_models('both')  -> boost_avg.slx, boost_cycle.slx
%     2. copies saved for older releases -> models_R2021b/*.slx
%     3. check_simulink_models       -> Simulink vs sim_boost vs Python traces
%     4. check_simscape_block_params -> real Simscape parameter names (info)
%     5. run_all_tests               -> the Octave-tested suite, now in MATLAB
%   Output: ci_output/ci_build_log.txt (full log) and ci_output/ci_summary.txt.
%   Errors at the end if a build, a model check or a test failed, so CI
%   shows red.

here = fileparts(mfilename('fullpath'));
cd(here);
addpath(here);
addpath(fullfile(here, 'tests'));
outdir = fullfile(here, 'ci_output');
if exist(outdir, 'dir') ~= 7
    mkdir(outdir);
end
logf = fullfile(outdir, 'ci_build_log.txt');
if exist(logf, 'file') == 2
    delete(logf);
end
diary(logf);
cleanup = onCleanup(@() diary('off'));

fprintf('ci_build on MATLAB %s\n', version);
v = ver;
for i = 1:numel(v)
    fprintf('  %-40s %s\n', v(i).Name, v(i).Version);
end

steps = cell(0, 3);   % name, pass, detail
models = {'boost_avg', 'boost_cycle'};

% ---- 1. build
fprintf('\n==== 1. build_boost_models\n');
built = false;
try
    files = build_boost_models('both', here);
    steps(end + 1, :) = {'build_boost_models', true, strjoin(files, ', ')};
    built = true;
catch err
    steps(end + 1, :) = {'build_boost_models', false, report(err)};
end

% ---- 2. copies for older MATLAB releases
fprintf('\n==== 2. export for older releases\n');
if built
    old = fullfile(here, 'models_R2021b');
    if exist(old, 'dir') ~= 7
        mkdir(old);
    end
    for i = 1:numel(models)
        try
            if bdIsLoaded(models{i})
                close_system(models{i}, 0);
            end
            load_system(fullfile(here, [models{i} '.slx']));
            target = fullfile(old, [models{i} '_R2021b.slx']);
            Simulink.exportToVersion(models{i}, target, 'R2021b');
            close_system(models{i}, 0);
            steps(end + 1, :) = {['export ' models{i} ' R2021b'], true, target}; %#ok<AGROW>
        catch err
            steps(end + 1, :) = {['export ' models{i} ' R2021b'], true, ['skipped: ' err.message]}; %#ok<AGROW>
        end
    end
end

% ---- 3. model checks
fprintf('\n==== 3. check_simulink_models\n');
if built
    try
        R = check_simulink_models(struct('folder', here));
        np = sum([R.pass]);
        bad = R(~[R.pass]);
        detail = sprintf('%d/%d rows pass', np, numel(R));
        for i = 1:numel(bad)
            detail = sprintf('%s\n      FAIL %s: %s', detail, bad(i).name, bad(i).detail);
        end
        steps(end + 1, :) = {'check_simulink_models', np == numel(R), detail};
    catch err
        steps(end + 1, :) = {'check_simulink_models', false, report(err)};
    end
end

% ---- 4. Simscape parameter names (information only)
fprintf('\n==== 4. check_simscape_block_params\n');
try
    check_simscape_block_params();
    steps(end + 1, :) = {'check_simscape_block_params', true, 'printed in the log'};
catch err
    steps(end + 1, :) = {'check_simscape_block_params', true, ['skipped: ' err.message]};
end

% ---- 5. test suite
fprintf('\n==== 5. run_all_tests\n');
try
    res = run_all_tests([], struct('quiet', true, 'no_error', true));
    st = {res.status};
    nf = sum(strcmp(st, 'FAIL'));
    detail = sprintf('%d PASS, %d FAIL, %d SKIP', sum(strcmp(st, 'PASS')), nf, sum(strcmp(st, 'SKIP')));
    for i = find(strcmp(st, 'FAIL'))
        detail = sprintf('%s\n      FAIL %s: %s', detail, res(i).name, res(i).message);
    end
    steps(end + 1, :) = {'run_all_tests', nf == 0, detail};
catch err
    steps(end + 1, :) = {'run_all_tests', false, report(err)};
end

bdclose('all');

% ---- summary
lines = {sprintf('ci_build summary (MATLAB %s)', version)};
for i = 1:size(steps, 1)
    lines{end + 1} = sprintf('%-4s %-34s %s', ternary(steps{i, 2}, 'PASS', 'FAIL'), steps{i, 1}, steps{i, 3}); %#ok<AGROW>
end
txt = strjoin(lines, newline);
fprintf('\n%s\n', txt);
fid = fopen(fullfile(outdir, 'ci_summary.txt'), 'w');
fprintf(fid, '%s\n', txt);
fclose(fid);
if ~all([steps{:, 2}])
    error('ci_build:failed', 'ci_build: at least one step failed, see ci_output/ci_summary.txt');
end
end

function s = report(err)
s = err.message;
for i = 1:numel(err.stack)
    s = sprintf('%s\n        at %s line %d', s, err.stack(i).name, err.stack(i).line);
    if i >= 6
        break;
    end
end
if ~isempty(err.cause)
    for i = 1:numel(err.cause)
        s = sprintf('%s\n        cause: %s', s, err.cause{i}.message);
    end
end
end

function r = ternary(c, a, b)
if c
    r = a;
else
    r = b;
end
end
