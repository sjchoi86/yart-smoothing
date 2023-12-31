addpath_yart
%% Smooth 1d trajectory using optimization
ccc
rng(0);

% Sample a trajectory
sec_max = 5; HZ = 50; L = round(sec_max*HZ); secs = linspace(0,sec_max,L)'; 
traj = chol(kernel_levse(secs,secs,ones(L,1),ones(L,1),[1,0.3])+1e-10*eye(L))'*randn(L,1);

% Run smoothing
pos_lower  = -3;
pos_upper  = +3;
vel_limit  = 10;
acc_limit  = 50;
jerk_limit = 500;
pos_init   = traj(1);
pos_final  = traj(end);
vel_init   = 0;
vel_final  = 0;
acc_init   = 0;
acc_final  = 0;
fade_dur   = 0.5;
fade_jerk_limit = 50;
VERBOSE = true;
[traj_smt,exit_flag] = optimization_based_smoothing_1d(...
    secs,traj,...
    'pos_lower',pos_lower,'pos_upper',pos_upper,...
    'vel_limit',vel_limit,...
    'acc_limit',acc_limit,...
    'jerk_limit',jerk_limit,...
    'pos_init',pos_init,...
    'pos_final',pos_final,...
    'vel_init',vel_init,'vel_final',vel_final,...
    'acc_init',acc_init,'acc_final',acc_final,...
    'fade_dur',fade_dur,'fade_jerk_limit',fade_jerk_limit,...
    'VERBOSE',VERBOSE);

% Check smoothing results
PLOT_FADE = true;
[pos,vel,acc,jerk,pos_smt,vel_smt,acc_smt,jerk_smt] = check_smoothing_results(...
    secs,traj,traj_smt,...
    'pos_lower',pos_lower,'pos_upper',pos_upper,...
    'vel_limit',vel_limit,...
    'acc_limit',acc_limit,...
    'jerk_limit',jerk_limit,...
    'pos_init',pos_init,...
    'pos_final',pos_final,...
    'vel_init',vel_init,'vel_final',vel_final,...
    'acc_init',acc_init,'acc_final',acc_final,...
    'fade_dur',fade_dur,'fade_jerk_limit',fade_jerk_limit,...
    'PLOT_FADE',PLOT_FADE,'VERBOSE',VERBOSE,...
    'lw',2,'afs',13,'tfs',13);

%%
