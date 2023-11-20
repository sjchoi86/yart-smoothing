addpath_yart
%% In-betweening 1D trajectories
ccc

rng(0); % fix seed
HZ = 30; dt = 1/HZ; len_param = 0.4;

% trajectory (from)
t_max = 5; L_fr = round(t_max*HZ); secs_fr = linspace(0,t_max,L_fr)';
K = kernel_levse(secs_fr,secs_fr,ones(L_fr,1),ones(L_fr,1),[1,len_param]);
traj_fr = chol(K+1e-10*eye(L_fr,L_fr))'*randn(L_fr,1);

% trajectory (to)
t_max = 3; L_to = round(t_max*HZ); secs_to = linspace(0,t_max,L_to)';
K = kernel_levse(secs_to,secs_to,ones(L_to,1),ones(L_to,1),[1,len_param]);
traj_to = chol(K+1e-10*eye(L_to,L_to))'*randn(L_to,1);

% Equality and inequality contraints
[vel_fr,acc_fr,jerk_fr] = get_vel_acc_jerk(secs_fr,traj_fr);
[vel_to,acc_to,jerk_to] = get_vel_acc_jerk(secs_to,traj_to);
pos_lower  = -3; % positional lowerbound
pos_upper  = +3; % positional upperbound
vel_limit  = [];
acc_limit  = 50;
jerk_limit = 100;
pos_init   = traj_fr(end);
pos_final  = traj_to(1);
vel_init   = vel_fr(end);
vel_final  = vel_to(1);
acc_init   = acc_fr(end);
acc_final  = acc_to(1);

% Run optimization
dur_tweening = 5; % base tweening duration
[secs_tween,traj_tween,exit_flag] = optimization_based_tweening_1d(...
    secs_fr,traj_fr,secs_to,traj_to,dur_tweening,...
    'pos_lower',pos_lower,'pos_upper',pos_upper,'vel_limit',vel_limit,...
    'acc_limit',acc_limit,'jerk_limit',jerk_limit,...
    'pos_init',pos_init,'pos_final',pos_final,'vel_init',vel_init,'vel_final',vel_final,...
    'acc_init',acc_init,'acc_final',acc_final);

% Check smoothing
check_tweening_results(...
    secs_fr,traj_fr,secs_tween,traj_tween,secs_to,traj_to,...
    'pos_lower',pos_lower,'pos_upper',pos_upper,'vel_limit',vel_limit,...
    'pos_init',pos_init,'pos_final',pos_final,...
    'vel_init',vel_init,'vel_final',vel_final,...
    'acc_init',acc_init,'acc_final',acc_final);

%%

