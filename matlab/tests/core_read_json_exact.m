function v = core_read_json_exact(fname)
%CORE_READ_JSON_EXACT  Small JSON reader with correctly rounded numbers.
%   v = core_read_json_exact(fname)
%   Octave 8.4's jsondecode can be 1 ulp off (e.g. 0.050400090500831156),
%   which breaks bit-exact comparisons; this parser converts every number
%   with str2double.  Objects -> struct (keys made valid like jsondecode:
%   'Settling (ms)' -> 'Settling_ms_'), arrays of numbers -> column vector,
%   arrays of objects with equal keys -> struct column, others -> cell.
txt = fileread(fname);
[v, pos] = parse_value(txt, skipws(txt, 1));
pos = skipws(txt, pos);
if pos <= numel(txt)
    error('core_read_json_exact:trailing', 'trailing characters at %d', pos);
end
end

function pos = skipws(txt, pos)
n = numel(txt);
while pos <= n && any(txt(pos) == sprintf(' \t\r\n'))
    pos = pos + 1;
end
end

function [v, pos] = parse_value(txt, pos)
c = txt(pos);
if c == '{'
    [v, pos] = parse_object(txt, pos);
elseif c == '['
    [v, pos] = parse_array(txt, pos);
elseif c == '"'
    [v, pos] = parse_string(txt, pos);
elseif strncmp(txt(pos:end), 'true', 4)
    v = true; pos = pos + 4;
elseif strncmp(txt(pos:end), 'false', 5)
    v = false; pos = pos + 5;
elseif strncmp(txt(pos:end), 'null', 4)
    v = []; pos = pos + 4;
elseif strncmp(txt(pos:end), 'NaN', 3)
    v = NaN; pos = pos + 3;
else
    tok = regexp(txt(pos:min(end, pos + 64)), '^-?(\d+\.?\d*([eE][-+]?\d+)?|Infinity)', 'match', 'once');
    if isempty(tok)
        error('core_read_json_exact:syntax', 'unexpected character at %d', pos);
    end
    if strcmp(tok, 'Infinity')
        v = Inf;
    elseif strcmp(tok, '-Infinity')
        v = -Inf;
    else
        v = str2double(tok);
    end
    pos = pos + numel(tok);
end
end

function [s, pos] = parse_string(txt, pos)
pos = pos + 1;
s = '';
while txt(pos) ~= '"'
    if txt(pos) == '\'
        pos = pos + 1;
        switch txt(pos)
            case 'n'
                s(end + 1) = char(10);
            case 't'
                s(end + 1) = char(9);
            case 'u'
                s(end + 1) = char(hex2dec(txt(pos + 1:pos + 4)));
                pos = pos + 4;
            otherwise
                s(end + 1) = txt(pos);
        end
    else
        s(end + 1) = txt(pos);
    end
    pos = pos + 1;
end
pos = pos + 1;
end

function [v, pos] = parse_object(txt, pos)
v = struct();
pos = skipws(txt, pos + 1);
if txt(pos) == '}'
    pos = pos + 1;
    return;
end
while true
    [key, pos] = parse_string(txt, skipws(txt, pos));
    pos = skipws(txt, pos);
    pos = pos + 1;                          % ':'
    [val, pos] = parse_value(txt, skipws(txt, pos));
    v.(valid_name(key)) = val;
    pos = skipws(txt, pos);
    if txt(pos) == ','
        pos = pos + 1;
    else
        pos = pos + 1;                      % '}'
        return;
    end
end
end

function [v, pos] = parse_array(txt, pos)
items = {};
pos = skipws(txt, pos + 1);
if txt(pos) == ']'
    v = zeros(0, 1);
    pos = pos + 1;
    return;
end
while true
    [items{end + 1}, pos] = parse_value(txt, skipws(txt, pos)); %#ok<AGROW>
    pos = skipws(txt, pos);
    if txt(pos) == ','
        pos = pos + 1;
    else
        pos = pos + 1;                      % ']'
        break;
    end
end
isnum = cellfun(@(x) isnumeric(x) && isscalar(x), items);
isst = cellfun(@isstruct, items);
if all(isnum)
    v = cell2mat(items(:));
elseif all(isst) && same_fields(items)
    v = vertcat(items{:});
else
    v = items(:);
end
end

function tf = same_fields(items)
f1 = fieldnames(items{1});
tf = true;
for i = 2:numel(items)
    if ~isequal(fieldnames(items{i}), f1)
        tf = false;
        return;
    end
end
end

function k = valid_name(key)
% like matlab.lang.makeValidName: drop whitespace (upper-casing a following
% lower-case letter), replace other invalid characters by '_'
k = '';
up = false;
for c = key
    if any(c == sprintf(' \t\r\n'))
        up = true;
        continue;
    end
    if up && c >= 'a' && c <= 'z'
        c = upper(c);
    end
    up = false;
    if ~(isletter(c) || (c >= '0' && c <= '9') || c == '_')
        c = '_';
    end
    k(end + 1) = c; %#ok<AGROW>
end
if isempty(k) || ~isletter(k(1))
    k = ['x' k];
end
end
