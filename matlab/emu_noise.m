function [noise, sd] = emu_noise(opts, N)
%EMU_NOISE  Sensor-noise samples for the 'noise' root Inport (4 x N).
%   [noise, sd] = emu_noise(opts, N)
%     opts  sim_boost options: noise_std_v, noise_std_i, noise_std_io,
%           noise_std_vin (default 0) and seed (default 0)
%     N     number of samples
%   noise(:,k) = [nVo; niL; nio; nVin] for sample k (t = (k-1)*Ts); rows of
%   inactive channels are 0.  Drawn exactly like sim_boost: rng(seed,
%   'twister'), z = randn(n_active, N), caller RNG state restored, so a
%   model fed with this matrix sees the same noise as sim_boost.
%   Simulink: in = in.setExternalInput([t, noise.']) or
%   timeseries(noise.', t) with t = (0:N-1)'*Ts (Inport 'noise', width 4).
if nargin < 1 || isempty(opts)
    opts = struct();
end
sd = [getf(opts, 'noise_std_v', 0), getf(opts, 'noise_std_i', 0), ...
      getf(opts, 'noise_std_io', 0), getf(opts, 'noise_std_vin', 0)];
seed = getf(opts, 'seed', 0);
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
end

function v = getf(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
