addpath_yart
%% In-betweening 1D trajectories
ccc

rng(4); % fix seed
HZ = 30; dt = 1/HZ; len_param = 0.4;

% trajectory (from)
t_max = 5; L_fr = round(t_max*HZ); secs_fr = linspace(0,t_max,L_fr)';
K = kernel_levse(secs_fr,secs_fr,ones(L_fr,1),ones(L_fr,1),[1,len_param]);
traj_fr = chol(K+1e-10*eye(L_fr,L_fr))'*randn(L_fr,1);

% trajectory (to)
t_max = 3; L_to = round(t_max*HZ); secs_to = linspace(0,t_max,L_to)';
K = kernel_levse(secs_to,secs_to,ones(L_to,1),ones(L_to,1),[1,len_param]);
traj_to = chol(K+1e-10*eye(L_to,L_to))'*randn(L_to,1);

% Run optimization
pos_lower = -3;
pos_upper = +3;
[secs_tween,traj_tween,exit_flag] = automated_tweening_1d(...
    secs_fr,traj_fr,secs_to,traj_to,...
    'pos_lower',pos_lower,'pos_upper',pos_upper,...
    'vel_limit_rate',1.0,'acc_limit_rate',0.5,'jerk_limit_rate',0.25,...
    'dur_tweenings',[1,2,5,10],'VERBOSE',1);

% Check smoothing
check_tweening_results(...
    secs_fr,traj_fr,secs_tween,traj_tween,secs_to,traj_to,...
    'pos_lower',pos_lower,'pos_upper',pos_upper);

%%
