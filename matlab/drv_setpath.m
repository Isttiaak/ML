function s = drv_setpath(s, path, value)
%DRV_SETPATH  Set a (possibly nested) struct field given as 'a.b.c'.
%   s = drv_setpath(s, 'mopso.n_iter', 4) is s.mopso.n_iter = 4, creating
%   missing intermediate structs.
parts = strsplit(path, '.');
s = setp(s, parts, value);
end

function s = setp(s, parts, value)
if isempty(s)
    s = struct();
end
if numel(parts) == 1
    s.(parts{1}) = value;
    return;
end
if isfield(s, parts{1}) && isstruct(s.(parts{1}))
    sub = s.(parts{1});
else
    sub = struct();
end
s.(parts{1}) = setp(sub, parts(2:end), value);
end
