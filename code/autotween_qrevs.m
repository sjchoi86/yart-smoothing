addpath_yart
%% Autotween two revolute joint trajectories
ccc

% Load motions
motion_idx_fr = 1;
motion_idx_to = 2;
[secs_fr,q_revs_fr,T_roots_fr,chain_robot] = load_motion(motion_idx_fr);
[secs_to,q_revs_to,T_roots_to,~] = load_motion(motion_idx_to);

% Smooth given motions ('q_revs_fr' and 'q_revs_to')
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
[q_revs_fr_smt,~] = optimization_based_smoothing_q_revs(...
    secs_fr,q_revs_fr,chain_robot,...
    'vel_limit',vel_limit,'acc_limit',acc_limit,'jerk_limit',jerk_limit,...
    'vel_init',vel_init,'vel_final',vel_final,...
    'acc_init',acc_init,'acc_final',acc_final,...
    'fade_dur',fade_dur,'fade_jerk_limit',fade_jerk_limit,...
    'CHECK_SC',CHECK_SC,'VERBOSE',VERBOSE);
[q_revs_to_smt,~] = optimization_based_smoothing_q_revs(...
    secs_to,q_revs_to,chain_robot,...
    'vel_limit',vel_limit,'acc_limit',acc_limit,'jerk_limit',jerk_limit,...
    'vel_init',vel_init,'vel_final',vel_final,...
    'acc_init',acc_init,'acc_final',acc_final,...
    'fade_dur',fade_dur,'fade_jerk_limit',fade_jerk_limit,...
    'CHECK_SC',CHECK_SC,'VERBOSE',VERBOSE);

% Automated in-betweening of revolute joint trajectories
vel_limit    = 360*D2R;
acc_limit    = 500*D2R;
jerk_limit   = 1000*D2R;
[secs_tween,q_revs_tween,secs_total,q_revs_total,exit_flags] = ...
    automated_tweening_q_revs( ... 
    secs_fr,q_revs_fr_smt,secs_to,q_revs_to_smt,chain_robot,'dur_tweenings',[2,5,10],...
    'vel_limit',vel_limit,'acc_limit',acc_limit,'jerk_limit',jerk_limit,...
    'pos_limit_margin',5*D2R,'PLOT_EACH_JOINT_TWEEN',0,'CHECK_SC',1,'VERBOSE',1);

%% Playback
ca;

L = size(secs_total,1); % motion length
chains = cell(1,L);
for tick = 1:L
    q = q_revs_total(tick,:);
    T_root = eye(4,4);
    chain_robot = update_chain_q_root_T(chain_robot,q,T_root);
    chain_robot_ground = move_chain_two_feet_on_ground(chain_robot);
    chains{tick} = chain_robot_ground; % append chain
end
axis_info = get_axis_info_from_chains(chains,'margin',0.5);

% Animate
for tick = 1:L
    sec = secs_total(tick);
    chain_robot_ground = chains{tick};
    % Check SC
    SC = check_sc(chain_robot_ground);
    if SC
        fprintf(2,"[%d/%d] self-collision occurred. \n",tick,L);
    end
    % Plot
    fig_idx = 1;
    fig = plot_chain(chain_robot_ground,'fig_idx',1,'fig_pos',[0.6,0.5,0.2,0.45],...
        'PLOT_ROTATE_AXIS',0,'PLOT_CAPSULE',1,...
        'axis_info',axis_info);
    title_str = sprintf("[%d/%d] SC:[%d]",tick,L,SC);
    plot_title(title_str,'fig_idx',fig_idx);
    drawnow; pause_invalid_handle(fig);
end

%%






