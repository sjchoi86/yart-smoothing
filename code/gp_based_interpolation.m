function traj_out = gp_based_interpolation(secs_in,traj_in,secs_out,varargin)
%
% Gaussian process based interpolation
%

% Parse options
iP = inputParser;
addParameter(iP,'hyp_mu',[1,0.1]);
addParameter(iP,'meas_noise_std',1e-6);
parse(iP,varargin{:});
hyp_mu         = iP.Results.hyp_mu;
meas_noise_std = iP.Results.meas_noise_std;

% Gaussian process based interpolation
n_in = size(secs_in,1);
n_out = size(secs_out,1);

k_out_in = kernel_levse(secs_out,secs_in,ones(n_out,1),ones(n_in,1),hyp_mu);
K_in_in = kernel_levse(secs_in,secs_in,ones(n_in,1),ones(n_in,1),hyp_mu);

n_in_half = round(n_in/2);
n_in_half_rest = n_in-n_in_half;
meas_noise_std_vec = meas_noise_std*...
    [linspace(0,1,n_in_half),linspace(1,0,n_in_half_rest)]';

meas_noise_std_diag = diag(meas_noise_std_vec);
traj_out = k_out_in / (K_in_in+meas_noise_std_diag) * ...
    (traj_in-mean(traj_in)) + mean(traj_in);
