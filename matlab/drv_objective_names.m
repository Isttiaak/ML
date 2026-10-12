function names = drv_objective_names(ctrl, cfg)
%DRV_OBJECTIVE_NAMES  Objective names returned by objective_boost.
%   names = drv_objective_names(ctrl, cfg)   (cell row)
name = drv_ctrl_name(ctrl);
if strcmp(cfg.objective_set, 'legacy')
    names = {'settling_ms', 'overshoot_pct', 'ss_error_V', 'load_reg_V'};
else
    names = {'startup_settling_ms', 'startup_overshoot_pct', 'event_peak_dev_V', 'recovery_ms'};
    if strcmp(name, 'fcsmpc') && cfg.mpc_fsw_objective
        names{end + 1} = 'fsw_kHz';
    end
    if isfield(cfg, 'extra_objectives') && ~isempty(cfg.extra_objectives)
        ex = cfg.extra_objectives;
        if ischar(ex)
            ex = {ex};
        end
        names = [names, ex(:).'];
    end
end
end
