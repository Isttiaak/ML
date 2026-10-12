function s = drv_merge(s, o, deep)
%DRV_MERGE  Copy the fields of struct o into struct s (o wins).
%   s = drv_merge(s, o)        shallow: o.f replaces s.f
%   s = drv_merge(s, o, true)  scalar sub-structs present in both are merged
%                              recursively instead of replaced
%   Empty o or non-struct o returns s unchanged.
if nargin < 3
    deep = false;
end
if isempty(o) || ~isstruct(o)
    return;
end
f = fieldnames(o);
for i = 1:numel(f)
    v = o.(f{i});
    if deep && isfield(s, f{i}) && isstruct(s.(f{i})) && isscalar(s.(f{i})) ...
            && isstruct(v) && isscalar(v)
        s.(f{i}) = drv_merge(s.(f{i}), v, true);
    else
        s.(f{i}) = v;
    end
end
end
