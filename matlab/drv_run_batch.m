function outs = drv_run_batch(ctrl, G, P, opts, cfg)
%DRV_RUN_BATCH  Simulate n gain sets with the configured backend.
%   outs = drv_run_batch(ctrl, G, P, opts, cfg)
%   G     n x ngains (sim_boost order), one candidate per row
%   opts  sim_boost options; opts.KPV / opts.KIV may be n x 1 vectors
%   cfg   study_config fields used: backend, batch_sim, simcfg
%   outs  1 x n struct array with the same fields for every backend:
%         t, Vo, iL, d (N x 1), Vo_min, Vo_max, iL_min, iL_max ([] for the
%         averaged plant), ok (false if the run threw), err (message)
%
%   backend 'matlab': averaged plant + cfg.batch_sim -> drv_sim_batch (all
%     rows in one vectorised run, bit-identical to sim_boost); otherwise
%     sim_boost row by row.
%   backend 'simulink': ONE call sim_boost_simulink(ctrl, G, P, ol,
%     cfg.simcfg) with ol a 1 x n cell of opts (each row its own scalar
%     KPV/KIV), so Fast Restart / parsim see the whole swarm at once.
%     cfg.simcfg.backend = 'emulate' runs the same path without Simulink.
n = size(G, 1);
KPV = vec(getf(opts, 'KPV', P.KPV), n);
KIV = vec(getf(opts, 'KIV', P.KIV), n);
backend = lower(getf(cfg, 'backend', 'matlab'));
plant = lower(getf(opts, 'plant', 'avg'));
empty1 = struct('t', [], 'Vo', [], 'iL', [], 'd', [], 'Vo_min', [], 'Vo_max', [], ...
    'iL_min', [], 'iL_max', [], 'ok', false, 'err', '');
outs = repmat(empty1, 1, n);

switch backend
    case 'matlab'
        if strcmp(plant, 'avg') && getf(cfg, 'batch_sim', true)
            o = opts;
            o.KPV = KPV;
            o.KIV = KIV;
            try
                b = drv_sim_batch(ctrl, G, P, o);
                for i = 1:n
                    outs(i).t = b.t;
                    outs(i).Vo = b.Vo(:, i);
                    outs(i).iL = b.iL(:, i);
                    outs(i).d = b.d(:, i);
                    outs(i).ok = true;
                end
            catch e
                [outs.err] = deal(e.message);
            end
        else
            for i = 1:n
                o = opts;
                o.KPV = KPV(i);
                o.KIV = KIV(i);
                try
                    r = sim_boost(ctrl, G(i, :), P, o);
                    outs(i) = fill(outs(i), r);
                catch e
                    outs(i).err = e.message;
                end
            end
        end
    case 'simulink'
        if exist('sim_boost_simulink', 'file') ~= 2
            error('drv_run_batch:simulink', ['backend ''simulink'' needs sim_boost_simulink.m ' ...
                'on the path (and Simulink); use cfg.backend = ''matlab''']);
        end
        % one call, one opts element per row (per-candidate KPV/KIV)
        ol = cell(1, n);
        for i = 1:n
            ol{i} = opts;
            ol{i}.KPV = KPV(i);
            ol{i}.KIV = KIV(i);
        end
        try
            r = sim_boost_simulink(ctrl, G, P, ol, getf(cfg, 'simcfg', struct()));
            for i = 1:n
                outs(i) = fill(outs(i), r(i));
                if isfield(r, 'ok') && ~isempty(r(i).ok)
                    outs(i).ok = outs(i).ok && logical(r(i).ok);
                end
                if isfield(r, 'error') && ~isempty(r(i).error)
                    outs(i).err = r(i).error;
                end
            end
        catch e
            [outs.err] = deal(e.message);
        end
    otherwise
        error('drv_run_batch:backend', 'backend must be ''matlab'' or ''simulink''');
end
end

function s = fill(s, r)
f = {'t', 'Vo', 'iL', 'd', 'Vo_min', 'Vo_max', 'iL_min', 'iL_max'};
for k = 1:numel(f)
    if isfield(r, f{k})
        s.(f{k}) = r.(f{k})(:);
    end
end
s.ok = ~isempty(s.Vo);
if ~s.ok
    s.err = 'no Vo returned';
end
end

function v = vec(v, n)
v = double(v(:));
if numel(v) == 1
    v = repmat(v, n, 1);
elseif numel(v) ~= n
    error('drv_run_batch:outer', 'KPV/KIV must be scalar or n x 1');
end
end

function v = getf(s, name, default)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
