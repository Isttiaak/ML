function [name, cid, ngains, has_outer] = drv_ctrl_name(ctrl)
%DRV_CTRL_NAME  Canonical controller name (same aliases as sim_boost).
%   [name, cid, ngains, has_outer] = drv_ctrl_name(ctrl)
%   name      'fcsmpc' | 'cascade_pi' | 'cascade_pid' | 'smc' | 'single_pi'
%   cid       1..5 in that order;  ngains  number of controller gains
%   has_outer true for the four controllers with the outer voltage PI
if ~ischar(ctrl)
    error('drv_ctrl_name:ctrl', 'controller name must be a char array');
end
switch lower(strrep(ctrl, '-', '_'))
    case {'fcsmpc', 'fcs_mpc', 'mpc'}
        name = 'fcsmpc'; cid = 1;
    case {'cascade_pi', 'pi'}
        name = 'cascade_pi'; cid = 2;
    case {'cascade_pid', 'pid'}
        name = 'cascade_pid'; cid = 3;
    case 'smc'
        name = 'smc'; cid = 4;
    case {'single_pi', 'spi'}
        name = 'single_pi'; cid = 5;
    otherwise
        error('drv_ctrl_name:ctrl', 'unknown controller ''%s''', ctrl);
end
nreq = [1 2 4 2 2];
ngains = nreq(cid);
has_outer = cid ~= 5;
end
