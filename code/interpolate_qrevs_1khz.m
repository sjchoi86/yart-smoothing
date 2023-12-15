addpath_yart
%% Interpolate joint revolute joints
ccc

% Load motion
motion_paths = dir_compact('../data/mhformer_*','VERBOSE',0);
n_motion = length(motion_paths); motion_idx = 1;
mat_path = [motion_paths(motion_idx).folder,'/',motion_paths(motion_idx).name];
l = load(mat_path);
chain_robot = l.chain_robot; secs = l.secs; L = size(secs,1);
q_revs = l.q_revs_robot_sch; T_roots = l.T_roots_robot_sch;
fprintf("We have [%d] motions.\n",n_motion);
fprintf("We will be using [%d/%d]-th motion from [%s]. \n",...
    motion_idx,n_motion,mat_path);

% Examine revolute joint trajectories
check_traj(secs,q_revs,'VERBOSE',1,'PLOT_FIGS',1,'lw',1,'traj_name','q_revs'); 

% Interpolation
HZ = round(length(secs)/(secs(end)-secs(1))); HZ_1khz = 1000;
L_1khz = round(L*HZ_1khz/HZ);
secs_1khz = linspace(secs(1),secs(end),L_1khz)';
q_revs_1khz = gp_based_interpolation(secs,q_revs,secs_1khz);

% Examine interpolated revolute joint trajectories
check_traj(secs_1khz,q_revs_1khz,'VERBOSE',1,'PLOT_FIGS',1,'lw',1,...
    'fig_idx_offset',4,'fig_pos_yoffset',0.25,'traj_name','q_revs_1khz'); 

%%
ca; clc;
axes_info = [0.12,0.12,0.83,0.8];
fig_idx = 1; USE_DRAGZOOM = 1;
fig_pos_yoffset = 0.0; afs = 10;
set_fig(figure(fig_idx),'pos',[0.0,0.65-fig_pos_yoffset,0.25,0.25],...
    'AXIS_EQUAL',0,'USE_DRAGZOOM',USE_DRAGZOOM,...
    'axes_info',axes_info,'ax_str','t [sec]','ay_str','x(t)','afs',afs);
plot(secs,q_revs,'-','Color','b');
plot(secs_1khz,q_revs_1khz,'-','Color','r');

%% Smooth first, then interpoate joint revolute joints
ccc

% Load motion
motion_paths = dir_compact('../data/mhformer_*','VERBOSE',0);
n_motion = length(motion_paths); motion_idx = 1;
mat_path = [motion_paths(motion_idx).folder,'/',motion_paths(motion_idx).name];
l = load(mat_path);
chain_robot = l.chain_robot; secs = l.secs; L = size(secs,1);
q_revs = l.q_revs_robot_sch; T_roots = l.T_roots_robot_sch;
fprintf("We have [%d] motions.\n",n_motion);
fprintf("We will be using [%d/%d]-th motion from [%s]. \n",...
    motion_idx,n_motion,mat_path);

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
q_revs_smt = optimization_based_smoothing_q_revs(...
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
    'CHECK_SC',CHECK_SC ...
);

% Examine revolute joint trajectories
check_traj(secs,q_revs_smt,'VERBOSE',1,'PLOT_FIGS',1,'lw',1,'traj_name','q_revs_smt'); 

% Interpolation
HZ = round(length(secs)/(secs(end)-secs(1))); HZ_1khz = 1000;
L_1khz = round(L*HZ_1khz/HZ);
secs_1khz = linspace(secs(1),secs(end),L_1khz)';
q_revs_smt_1khz = gp_based_interpolation(secs,q_revs_smt,secs_1khz,...
    'hyp_mu',[1,0.2],'meas_noise_std',1e-4);

% Examine interpolated revolute joint trajectories
check_traj(secs_1khz,q_revs_smt_1khz,'VERBOSE',1,'PLOT_FIGS',1,'lw',1,...
    'fig_idx_offset',4,'fig_pos_yoffset',0.25,'traj_name','q_revs_smt_1khz'); 

%%


