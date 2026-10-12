function out = drv_sim_batch(ctrl, G, P, opts)
%DRV_SIM_BATCH  sim_boost for many gain sets at once (averaged plant only).
%   out = drv_sim_batch(ctrl, G, P, opts)
%   G    : n x ngains matrix, one candidate per row (notebook gain order).
%   opts : the sim_boost options; opts.KPV and opts.KIV may also be n x 1
%          vectors (one outer-loop gain per candidate).  Only the averaged
%          plant is supported (opts.plant 'avg'); use sim_boost for the
%          switched plant.
%   out  : t (N x 1), R, Vin (N x 1), Vo, iL, d (N x n), info.
%   The per-sample order and every floating-point expression are those of
%   sim_boost, vectorised over the candidates, so each column equals
%   sim_boost(ctrl, G(i,:), P, opts) (tests/test_drivers checks this; SMC
%   may differ by an ulp if a platform uses a SIMD tanh for arrays).
%   Noise is drawn exactly like sim_boost (same seed -> the same noise
%   sequence for every candidate).

if nargin < 3 || isempty(P)
    P = boost_params();
end
if nargin < 4 || isempty(opts)
    opts = struct();
end
name = drv_ctrl_name(ctrl);
cid = find(strcmp(name, {'fcsmpc', 'cascade_pi', 'cascade_pid', 'smc', 'single_pi'}));
nreq = [1 2 4 2 2];
if isempty(G)
    G = P.gains_baseline.(name);
end
G = double(G);
if size(G, 2) ~= nreq(cid)
    error('drv_sim_batch:gains', '%s needs %d gains per row, got %d', name, nreq(cid), size(G, 2));
end
n = size(G, 1);
plant = lower(gopt(opts, 'plant', 'avg'));
if ~strcmp(plant, 'avg')
    error('drv_sim_batch:plant', 'only the averaged plant is vectorised; use sim_boost');
end

Ts = P.Ts;
Vref = P.Vref;
D_ss = P.D_ss;
DMIN = P.DMIN;
DMAX = P.DMAX;
KPV = colvec(gopt(opts, 'KPV', P.KPV), n);
KIV = colvec(gopt(opts, 'KIV', P.KIV), n);
IL_MAX = gopt(opts, 'IL_MAX', P.IL_MAX);
T_end = gopt(opts, 'T_end', P.T_end);
N = fix(T_end / Ts);
t = (0:N - 1)' * Ts;

lspec = gopt(opts, 'load', []);
if isempty(lspec)
    lspec = struct('kind', 'step', 'R0', P.R0, 'R_new', P.R1, 't_step', P.t_step);
end
if ~isfield(lspec, 'R0')
    lspec.R0 = P.R0;
end
Rk = load_profile(t, lspec);
vspec = gopt(opts, 'vin', []);
if isempty(vspec)
    vspec = struct('kind', 'const');
end
if ~isfield(vspec, 'Vin')
    vspec.Vin = P.Vin;
end
Vk = vin_profile(t, vspec);

switch lower(gopt(opts, 'ff_mode', 'oracle'))
    case 'oracle'
        fid = 1;
    case 'measured'
        fid = 2;
    case 'none'
        fid = 3;
    otherwise
        error('drv_sim_batch:ff_mode', 'ff_mode must be oracle, measured or none');
end
delay = round(gopt(opts, 'delay', 0));
Pp = gopt(opts, 'Pplant', P);

sd = [gopt(opts, 'noise_std_v', 0), gopt(opts, 'noise_std_i', 0), ...
      gopt(opts, 'noise_std_io', 0), gopt(opts, 'noise_std_vin', 0)];
bits = gopt(opts, 'adc_bits', 0);
rV = gopt(opts, 'Vo_adc_range', P.Vo_adc_range);
rI = gopt(opts, 'iL_adc_range', P.iL_adc_range);
rIo = gopt(opts, 'io_adc_range', P.io_adc_range);
rVin = gopt(opts, 'vin_adc_range', P.vin_adc_range);
seed = gopt(opts, 'seed', 0);
noise = zeros(4, N);
act = find(sd > 0);
if ~isempty(act)
    rng_saved = rng;
    rng(seed, 'twister');
    z = randn(numel(act), N);
    rng(rng_saved);
    for j = 1:numel(act)
        noise(act(j), :) = sd(act(j)) * z(j, :);
    end
end
sense_on = (bits > 0) || ~isempty(act);

iL = zeros(N, n);
Vo = zeros(N, n);
d = zeros(N, n);
iL(1, :) = gopt(opts, 'iL0', 0.0);
Vo(1, :) = gopt(opts, 'Vo0', P.Vin);

z0 = zeros(n, 1);
int_v = z0; int_i = z0; ei_prev = z0; d_filt = z0; u_prev = z0;
if cid == 1
    d_init = 0;
else
    d_init = D_ss;
end
dbuf = d_init * ones(n, delay);
g1 = G(:, 1);
if size(G, 2) >= 2
    g2 = G(:, 2);
end
if cid == 3
    g3 = G(:, 3);
    g4 = G(:, 4);
    alpha = g4 ./ (g4 + Ts);
end
L_model = P.L;
Vin_nom = P.Vin;
Lp = Pp.L;
Cp = Pp.C;

for k = 1:N
    R = Rk(k);
    Vin_k = Vk(k);
    Vo_k = Vo(k, :).';
    iL_k = iL(k, :).';
    if sense_on
        Vo_m = sense_vec(Vo_k, noise(1, k), bits, rV(1), rV(2));
        iL_m = sense_vec(iL_k, noise(2, k), bits, rI(1), rI(2));
    else
        Vo_m = Vo_k;
        iL_m = iL_k;
    end

    if cid ~= 5
        if fid == 1
            IL_ff = (Vref * Vref) / (Vin_k * R);
            Vin_model = Vin_k;
        elseif fid == 2
            io_m = sense_vec(Vo_k / R, noise(3, k), bits, rIo(1), rIo(2));
            vin_m = sense_vec(Vin_k, noise(4, k), bits, rVin(1), rVin(2));
            IL_ff = Vref * io_m / max(vin_m, 1e-3);
            Vin_model = vin_m;
        else
            IL_ff = 0.0;
            Vin_model = Vin_nom;
        end
        % ctrl_outer_pi
        ev = Vref - Vo_m;
        IL_raw = IL_ff + KPV .* ev + KIV .* int_v;
        IL_ref = clip(IL_raw, 0, IL_MAX);
        int_v = int_v + ev * Ts + (IL_ref - IL_raw) ./ (KIV + 1e-12);
    end

    switch cid
        case 1      % ctrl_fcsmpc
            J = Inf(n, 1);
            dc = zeros(n, 1);
            for uc = 0:1
                diL = (Vin_model - (1 - uc) * Vo_m) / L_model;
                iL_p = iL_m + Ts * diL;
                e = iL_p - IL_ref;
                du = uc - u_prev;
                Jc = e .* e + g1 .* (du .* du);
                take = Jc < J;
                J(take) = Jc(take);
                dc(take) = uc;
            end
            u_prev = dc;
        case 2      % ctrl_inner_pi
            ei = IL_ref - iL_m;
            d_raw = D_ss + g1 .* ei + g2 .* int_i;
            dc = clip(d_raw, DMIN, DMAX);
            int_i = int_i + (ei * Ts + (dc - d_raw) ./ (g2 + 1e-12));
        case 3      % ctrl_inner_pid
            ei = IL_ref - iL_m;
            d_filt = alpha .* d_filt + (1 - alpha) .* (ei - ei_prev) / Ts;
            d_raw = D_ss + g1 .* ei + g2 .* int_i + g3 .* d_filt;
            dc = clip(d_raw, DMIN, DMAX);
            int_i = int_i + (ei * Ts + (dc - d_raw) ./ (g2 + 1e-12));
            ei_prev = ei;
        case 4      % ctrl_smc
            sigma = IL_ref - iL_m;
            dc = clip(D_ss + g1 .* tanh(sigma ./ g2), DMIN, DMAX);
        otherwise   % ctrl_single_pi
            ev = Vref - Vo_m;
            d_raw = D_ss + g1 .* ev + g2 .* int_v;
            dc = clip(d_raw, DMIN, DMAX);
            int_v = int_v + (ev * Ts + (dc - d_raw) ./ (g2 + 1e-12));
    end

    if delay == 0
        da = dc;
    else
        da = dbuf(:, 1);
        dbuf = [dbuf(:, 2:end), dc];
    end
    d(k, :) = da.';

    if k < N
        % plant_avg_step
        diL = (Vin_k - (1 - da) .* Vo_k) / Lp;
        dVo = ((1 - da) .* iL_k - Vo_k / R) / Cp;
        iL(k + 1, :) = (iL_k + Ts * diL).';
        Vo(k + 1, :) = (Vo_k + Ts * dVo).';
    end
end

out.t = t;
out.R = Rk;
out.Vin = Vk;
out.Vo = Vo;
out.iL = iL;
out.d = d;
out.info = struct('ctrl', name, 'n', n, 'N', N, 'Ts', Ts, 'delay', delay, 'seed', seed);
end

function y = clip(x, lo, hi)
% same result as the if/elseif clamps of the ctrl_* functions (lo < hi)
y = x;
y(x > hi) = hi;
y(x < lo) = lo;
end

function y = sense_vec(x, noise_sample, bits, lo, hi)
% element-wise sense_adc (same expressions)
y = x + noise_sample;
if bits > 0
    levels = 2^bits;
    q = round_half_even((y - lo) / (hi - lo) * (levels - 1));
    q(q < 0) = 0;
    q(q > levels - 1) = levels - 1;
    y = lo + q / (levels - 1) * (hi - lo);
end
end

function v = colvec(v, n)
v = double(v(:));
if numel(v) == 1
    v = repmat(v, n, 1);
elseif numel(v) ~= n
    error('drv_sim_batch:outer', 'KPV/KIV must be scalar or have one value per candidate');
end
end

function v = gopt(s, name, default)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
