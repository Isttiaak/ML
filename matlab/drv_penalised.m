function mask = drv_penalised(F, cfg)
%DRV_PENALISED  Rows of objective_boost output that carry a penalty value.
%   mask = drv_penalised(F, cfg)   (n x 1 logical)
%   corrected: never settled (settling >= penalty.settling_ms), never
%   recovered (recovery >= penalty.recovery_ms) or diverged (overshoot >=
%   penalty.overshoot_pct).  legacy: only the non-finite/diverged penalty
%   row (overshoot >= penalty.overshoot_pct); the notebook's 50 ms settling
%   sentinel is kept as a normal value there.
pen = cfg.penalty;
if isempty(F)
    mask = false(0, 1);
    return;
end
mask = F(:, 2) >= pen.overshoot_pct;
if ~strcmp(cfg.objective_set, 'legacy')
    mask = mask | F(:, 1) >= pen.settling_ms | F(:, 4) >= pen.recovery_ms;
end
end
