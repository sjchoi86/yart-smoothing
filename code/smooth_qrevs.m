addpath_yart
%% Smooth revolute joint trajectories using optimization
ccc

% Load motion
motion_paths = dir_compact('../data/mhformer_*','VERBOSE',0);
n_motion = length(motion_paths); motion_idx = 1;
mat_path = [motion_paths(motion_idx).folder,'/',motion_paths(motion_idx).name];
l = load(mat_path);
chain_robot = l.chain_robot; secs = l.secs; 
q_revs = l.q_revs_robot_sch; T_roots = l.T_roots_robot_sch;
fprintf("We have [%d] motions.\n",n_motion);
fprintf("We will be using [%d/%d]-th motion from [%s]. \n",...
    motion_idx,n_motion,mat_path);

% Examine revolute joint trajectories
% check_traj(secs,q_revs,'VERBOSE',false,'lw',1); pause; ca;

% Run smoothing
vel_limit           = 360*D2R;
acc_limit           = 1000*D2R;
jerk_limit          = 10000*D2R;
vel_init            = 0;
vel_final           = 0;
acc_init            = 0;
acc_final           = 0;
fade_dur            = 1.0;
fade_jerk_limit     = 500*D2R;
CHECK_SC            = true;
VERBOSE             = true;
[q_revs_smt,exit_flags] = optimization_based_smoothing_q_revs(...
    secs,q_revs,chain_robot,...
    'vel_limit',vel_limit,...
    'acc_limit',acc_limit,...
    'jerk_limit',jerk_limit,...
    'vel_init',vel_init,....
    'vel_final',vel_final,...
    'acc_init',acc_init,...
    'acc_final',acc_final,...
    'fade_dur',fade_dur,...
    'fade_jerk_limit',fade_jerk_limit,...
    'CHECK_SC',CHECK_SC, ...
    'VERBOSE',VERBOSE...
);

% Check smoothing results
ca; % close all
PLOT_FADE = true;
[pos,vel,acc,jerk,pos_smt,vel_smt,acc_smt,jerk_smt] = ...
    check_smoothing_results(...
    secs,q_revs,q_revs_smt,...
    'vel_limit',vel_limit,...
    'acc_limit',acc_limit,...
    'jerk_limit',jerk_limit,...
    'vel_init',vel_init,'vel_final',vel_final,...
    'acc_init',acc_init,'acc_final',acc_final,...
    'fade_dur',fade_dur,'fade_jerk_limit',fade_jerk_limit,...
    'PLOT_FADE',PLOT_FADE,'VERBOSE',1,...
    'lw',1,'afs',13,'tfs',13);

%%
