function s = drv_latex_escape(s)
%DRV_LATEX_ESCAPE  Escape LaTeX special characters in a plain-text cell.
%   s = drv_latex_escape(s)   handles \ % _ & # $ { } ~ ^ (one pass, so
%   the escapes themselves are never escaped again).
if isempty(s)
    s = '';
    return;
end
s = char(s);
out = cell(1, numel(s));
for i = 1:numel(s)
    ch = s(i);
    switch ch
        case '\'
            out{i} = '\textbackslash{}';
        case {'%', '_', '&', '#', '$', '{', '}'}
            out{i} = ['\' ch];
        case '~'
            out{i} = '\textasciitilde{}';
        case '^'
            out{i} = '\textasciicircum{}';
        otherwise
            out{i} = ch;
    end
end
s = [out{:}];
end
