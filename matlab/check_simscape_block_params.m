function info = check_simscape_block_params(names)
%CHECK_SIMSCAPE_BLOCK_PARAMS  Print the programmatic parameter names of the Simscape blocks used by the guide.
%   info = check_simscape_block_params()
%   info = check_simscape_block_params({'Inductor', 'PWM Generator'})   only these
%   Loads the Simscape libraries (ee_lib = Simscape Electrical, fl_lib =
%   Simscape Foundation, nesl_utility = Simscape utilities), copies each block
%   of the power stage into a scratch model (never saved) and prints
%     - the library path that worked,
%     - every dialog parameter: programmatic name = current value  [prompt],
%       plus the choices of popup parameters,
%   so that the labels in your release can be matched to the names used in
%   set_param (e.g. the inductance parameter of the Foundation Inductor is
%   usually 'l' with unit 'l_unit').  Blocks that are not found at the
%   expected path are searched by name in the loaded libraries.
%   info: struct array with fields name, path, params (struct of name/value
%   pairs as char) and ok.
%   UNTESTED here (no MATLAB): needs Simulink + Simscape (+ Simscape
%   Electrical for ee_lib blocks).

cand = { ...
    'Ideal Semiconductor Switch', {'ee_lib/Semiconductors & Converters/Ideal Semiconductor Switch'}; ...
    'Diode', {'fl_lib/Electrical/Electrical Elements/Diode', 'ee_lib/Semiconductors & Converters/Diode'}; ...
    'Inductor', {'fl_lib/Electrical/Electrical Elements/Inductor', 'ee_lib/Passive/Inductor'}; ...
    'Capacitor', {'fl_lib/Electrical/Electrical Elements/Capacitor', 'ee_lib/Passive/Capacitor'}; ...
    'Variable Resistor', {'fl_lib/Electrical/Electrical Elements/Variable Resistor', ...
        'ee_lib/Passive/Variable Resistor'}; ...
    'Resistor', {'fl_lib/Electrical/Electrical Elements/Resistor', 'ee_lib/Passive/Resistor'}; ...
    'Electrical Reference', {'fl_lib/Electrical/Electrical Elements/Electrical Reference'}; ...
    'Controlled Voltage Source', {'fl_lib/Electrical/Electrical Sources/Controlled Voltage Source'}; ...
    'DC Voltage Source', {'fl_lib/Electrical/Electrical Sources/DC Voltage Source'}; ...
    'Current Sensor', {'fl_lib/Electrical/Electrical Sensors/Current Sensor'}; ...
    'Voltage Sensor', {'fl_lib/Electrical/Electrical Sensors/Voltage Sensor'}; ...
    'Simulink-PS Converter', {'nesl_utility/Simulink-PS Converter'}; ...
    'PS-Simulink Converter', {'nesl_utility/PS-Simulink Converter'}; ...
    'Solver Configuration', {'nesl_utility/Solver Configuration'}; ...
    'PWM Generator', {'ee_lib/Control/Pulse Width Modulation/PWM Generator', ...
        'ee_lib/Control/PWM Generator'}};
if nargin >= 1 && ~isempty(names)
    if ischar(names)
        names = {names};
    end
    keep = ismember(cand(:, 1), names);
    cand = cand(keep, :);
end
if isempty(which('load_system'))
    error('check_simscape_block_params:simulink', 'Simulink is required');
end
try
    fprintf('MATLAB %s\n', version);
    v = ver('simscape');
    if ~isempty(v)
        fprintf('Simscape %s\n', v(1).Version);
    end
    v = ver('sps');
    if ~isempty(v)
        fprintf('Simscape Electrical %s\n', v(1).Version);
    end
catch
end

libs = {'ee_lib', 'fl_lib', 'nesl_utility'};
loaded = {};
for i = 1:numel(libs)
    try
        load_system(libs{i});
        loaded{end + 1} = libs{i}; %#ok<AGROW>
        fprintf('library %-13s loaded\n', libs{i});
    catch err
        fprintf('library %-13s NOT available (%s)\n', libs{i}, err.message);
    end
end

scratch = 'simscape_param_probe';
if bdIsLoaded(scratch)
    close_system(scratch, 0);
end
new_system(scratch);
cleanup = onCleanup(@() close_quiet(scratch));

info = struct('name', cand(:, 1), 'path', '', 'params', [], 'ok', false);
for i = 1:size(cand, 1)
    nm = cand{i, 1};
    fprintf('\n=== %s ===\n', nm);
    dst = [scratch '/' regexprep(nm, '[^A-Za-z0-9]', '_')];
    used = '';
    paths = cand{i, 2};
    for j = 1:numel(paths)
        try
            add_block(paths{j}, dst, 'Position', [100 100 160 160]);
            used = paths{j};
            break;
        catch
        end
    end
    if isempty(used)
        hits = search_libs(loaded, nm);
        for j = 1:numel(hits)
            try
                add_block(hits{j}, dst, 'Position', [100 100 160 160]);
                used = hits{j};
                break;
            catch
            end
        end
        if numel(hits) > 1
            fprintf('  other matches: %s\n', strjoin(hits, ' | '));
        end
    end
    if isempty(used)
        fprintf('  NOT FOUND: tried %s; search the Library Browser for ''%s''\n', strjoin(paths, ' | '), nm);
        continue;
    end
    fprintf('  library path: %s\n', used);
    info(i).path = used;
    info(i).ok = true;
    info(i).params = print_params(dst);
end
fprintf('\nUse these names with set_param(blk, name, value); unit parameters end in _unit.\n');
end

% ------------------------------------------------------------------------
function p = print_params(blk)
p = struct();
dp = struct();
try
    dp = get_param(blk, 'DialogParameters');
catch
end
if isempty(dp)
    dp = struct();
end
fn = fieldnames(dp);
if isempty(fn)
    % fall back to the mask parameters
    try
        mn = get_param(blk, 'MaskNames');
        mv = get_param(blk, 'MaskValues');
        for k = 1:numel(mn)
            fprintf('  %-28s = %s\n', mn{k}, val2str(mv{k}));
            p.(mn{k}) = val2str(mv{k});
        end
    catch err
        fprintf('  (no dialog parameters readable: %s)\n', err.message);
    end
    return;
end
for k = 1:numel(fn)
    val = '';
    try
        val = val2str(get_param(blk, fn{k}));
    catch
        val = '<unreadable>';
    end
    prompt = '';
    try
        prompt = dp.(fn{k}).Prompt;
    catch
    end
    fprintf('  %-28s = %-28s [%s]\n', fn{k}, val, prompt);
    try
        if strcmpi(dp.(fn{k}).Type, 'enum') && ~isempty(dp.(fn{k}).Enum)
            en = dp.(fn{k}).Enum;
            fprintf('  %-28s   choices: %s\n', '', strjoin(en(:)', ' | '));
        end
    catch
    end
    try
        p.(fn{k}) = val;
    catch
    end
end
end

function hits = search_libs(libs, name)
hits = {};
for i = 1:numel(libs)
    try
        h = find_system(libs{i}, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Name', name);
        hits = [hits; h(:)]; %#ok<AGROW>
    catch
    end
end
if isempty(hits)
    return;
end
% shortest paths first (library entries, not blocks inside implementations)
nd = cellfun(@(s) sum(s == '/'), hits);
[~, o] = sort(nd);
hits = hits(o)';
end

function s = val2str(v)
if ischar(v)
    s = v;
elseif isnumeric(v) || islogical(v)
    s = mat2str(v);
elseif iscell(v)
    s = ['{' strjoin(cellfun(@val2str, v, 'UniformOutput', false), ', ') '}'];
else
    s = ['<' class(v) '>'];
end
end

function close_quiet(mdl)
try
    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end
catch
end
end
