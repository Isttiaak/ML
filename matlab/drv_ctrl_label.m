function lbl = drv_ctrl_label(ctrl)
%DRV_CTRL_LABEL  Display name of a controller for tables and figures.
switch drv_ctrl_name(ctrl)
    case 'fcsmpc'
        lbl = 'FCS-MPC';
    case 'cascade_pi'
        lbl = 'Cascade PI';
    case 'cascade_pid'
        lbl = 'Cascade PID';
    case 'smc'
        lbl = 'SMC';
    otherwise
        lbl = 'Single-loop PI';
end
end
