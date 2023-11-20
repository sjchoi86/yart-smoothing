addpath_yart
addpath_rmr
%%
%
% Smooth (+fading) all motions and auto-tween smoothed motions
%
%
ccc

% Configuration
RE_SMT       = false;
RE_SMT_CH    = false;
RE_TWEEN     = false;
n_checkpoint = 5;

% Run motion smoothing and collision handling
motion_paths = dir_compact('../data/mhformer_*','VERBOSE',0);
n_motion = length(motion_paths);
for i_idx = 1:n_motion % for all motions

    % Get the original and smoothed motion paths
    mat_path = [motion_paths(i_idx).folder,'/',motion_paths(i_idx).name];
    fprintf("[%d/%d] %s \n",i_idx,n_motion,mat_path);

    % Smooth (+fading) all motions with caching
    smt_path = [motion_paths(i_idx).folder,'_smt/',motion_paths(i_idx).name];
    if exist(smt_path,'file') && (RE_SMT==0)
        l = load(smt_path);
        fprintf('[%s] loaded.\n',smt_path);
        secs = l.secs; q_revs = l.q_revs; q_revs_smt = l.q_revs_smt;
        T_roots = l.T_roots; exit_flags = l.exit_flags; chain_robot = l.chain_robot;
    else
        l = load(mat_path);
        secs = l.secs; q_revs = l.q_revs_robot_sch; chain_robot = l.chain_robot;
        T_roots = l.T_roots_robot_sch;tatus
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
        % Save to 'cache_path'
        [p,~,~] = fileparts(smt_path);
        make_dir_if_not_exist(p)
        save(smt_path,'secs','q_revs','q_revs_smt','T_roots','exit_flags','chain_robot');
        fprintf(2,'[%s] saved.\n',smt_path);
    end

    % Additional self-collision handling using 'q_revs_smt'
    smt_cf_path = [motion_paths(i_idx).folder,'_smt_cf/',motion_paths(i_idx).name];
    if exist(smt_cf_path,'file') && (RE_SMT_CH==0)
        l = load(smt_cf_path);
        fprintf('[%s] loaded.\n',smt_cf_path);
        secs = l.secs; q_revs = l.q_revs; q_revs_smt = l.q_revs_smt;
        q_revs_smt_cf = l.q_revs_smt_cf; T_roots_smt_cf = l.T_roots_smt_cf;
        T_roots = l.T_roots; exit_flags = l.exit_flags; chain_robot = l.chain_robot;
    else
        L = length(secs);
        chains = cell(1,L);
        SC_occurred = false;
        for tick = 1:L
            q = q_revs_smt(tick,:);
            T_root = T_roots{tick};
            chain_robot = update_chain_q_root_T(chain_robot,q,T_root);
            chain_robot_ground = move_chain_two_feet_on_ground(chain_robot);
            SC = check_sc(chain_robot_ground);
            if SC, SC_occurred = true; end
            chains{tick} = chain_robot_ground; % append chain
        end
        axis_info = get_axis_info_from_chains(chains,'margin',0.5);
        ca; % close all
        if SC_occurred % animate only the self-collision occurred motion
            [q_revs_smt_cf,T_roots_smt_cf] = smoothing_and_collision_handling(...
                chain_robot,secs,T_roots,q_revs_smt,...
                'PLOT_CHAIN_ZERO_POSE',0,'ANIMATE_SC_CHECK',0,'ANIMATE_SC_HANDLING',0,...
                'PLOT_CH_SMT_TRAJ',0,'VERBOSE',1,'SAVE_MAT',0);
        else
            q_revs_smt_cf = q_revs_smt;
            T_roots_smt_cf = T_roots;
        end
        % Save
        [p,~,~] = fileparts(smt_cf_path);
        make_dir_if_not_exist(p)
        save(smt_cf_path,'secs','q_revs','q_revs_smt','T_roots','exit_flags','chain_robot',...
            'q_revs_smt_cf','T_roots_smt_cf');
        fprintf(2,'[%s] saved.\n',smt_cf_path);
    end
end % for i_idx = 1:n_motion % for all motions

% Run in-betweening between motions
motion_paths = dir_compact('../data_smt_cf/mhformer_*','VERBOSE',0);
n_motion = length(motion_paths);
for i_idx = 1:n_motion % fr motion
    motion_name_fr = strrep(motion_paths(i_idx).name,'.mat','');
    smt_cf_path = [motion_paths(i_idx).folder,'/',motion_paths(i_idx).name];
    l = load(smt_cf_path); secs_fr = l.secs;
    chain_robot = l.chain_robot; q_revs_fr = l.q_revs_smt_cf; T_roots_fr = l.T_roots_smt_cf;
    L_fr = size(q_revs_fr,1);

    for check_idx = 1:n_checkpoint % fr-checkpoint
        L_fr_check = round(check_idx/n_checkpoint*L_fr);
        secs_fr_check = secs_fr(1:L_fr_check);
        q_revs_fr_check = q_revs_fr(1:L_fr_check,:);

        for j_idx = 1:n_motion % to motion
            motion_name_to = strrep(motion_paths(j_idx).name,'.mat','');
            smt_cf_path = [motion_paths(j_idx).folder,'/',motion_paths(j_idx).name];
            l = load(smt_cf_path); secs_to = l.secs;
            q_revs_to = l.q_revs_smt_cf; T_roots_to = l.T_roots_smt_cf;
            fprintf("[%d/%d] [%d/%d] [%d/%d] tweening fr:[%s]@tick:[%d/%d] to:[%s] \n",...
                i_idx,n_motion,check_idx,n_checkpoint,j_idx,n_motion,...
                motion_name_fr,L_fr_check,L_fr,motion_name_to);

            % Run automated in-betweening of revolute joint trajectories
            tween_path = sprintf('../data_tween/fr_%s@tick_%d_to_%s.mat',...
                motion_name_fr,L_fr_check,motion_name_to);
            if exist(tween_path,'file') && (RE_TWEEN==0)
                l = load(tween_path);
                fprintf('[%s] loaded.\n',tween_path);
                secs_tween = l.secs_tween;
                q_revs_tween = l.q_revs_tween;
            else
                vel_limit    = 360*D2R;
                acc_limit    = 500*D2R;
                jerk_limit   = 1000*D2R;
                [secs_tween,q_revs_tween,secs_total,q_revs_total,exit_flags] = ...
                    automated_tweening_q_revs( ...
                    secs_fr_check,q_revs_fr_check,secs_to,q_revs_to,chain_robot,...
                    'dur_tweenings',[1,2,5],...
                    'vel_limit',vel_limit,'acc_limit',acc_limit,'jerk_limit',jerk_limit,...
                    'pos_limit_margin',5*D2R,'PLOT_EACH_JOINT_TWEEN',0,'CHECK_SC',1,'VERBOSE',1);
                % Save
                [p,~,~] = fileparts(tween_path);
                make_dir_if_not_exist(p);
                save(tween_path,...
                    'secs_tween','q_revs_tween','secs_total','q_revs_total','exit_flags');
                fprintf(2,'[%s] saved.\n',tween_path);
            end

        end % for j_idx = 1:n_motion % to motion
    end % for check_idx = 1:n_checkpoint % fr-checkpoint
end % for i_idx = 1:n_motion % fr motion

%% Check inbetweening between motions
ccc

motion_paths = dir_compact('../data_smt_cf/mhformer_*','VERBOSE',0);
n_motion = length(motion_paths);
n_checkpoint = 5;

% Select source motion, checkpoint index, and target motion
motion_idx_fr = 1; % randi([1,n_motion])
check_idx     = 4; % randi([1,n_checkpoint])
motion_idx_to = 3; % randi([1,n_motion])

% Load related information

% Source motion
motion_name_fr = strrep(motion_paths(motion_idx_fr).name,'.mat','');
fr_path = [motion_paths(motion_idx_fr).folder,'/',motion_paths(motion_idx_fr).name];
l = load(fr_path); secs_fr = l.secs; chain_robot = l.chain_robot; q_revs_fr = l.q_revs_smt_cf; 
dt = secs_fr(2)-secs_fr(1); HZ = round(1/dt);
T_roots_fr = l.T_roots_smt_cf; L_fr = size(q_revs_fr,1);

% Checkpoint
L_fr_check = round(check_idx/n_checkpoint*L_fr);

% Target motion
motion_name_to = strrep(motion_paths(motion_idx_to).name,'.mat','');
to_path = [motion_paths(motion_idx_to).folder,'/',motion_paths(motion_idx_to).name];
l = load(to_path); secs_to = l.secs; q_revs_to = l.q_revs_smt_cf; 
T_roots_to = l.T_roots_smt_cf; L_to = size(q_revs_to,1);

% Tweening motion
tween_path = sprintf('../data_tween/fr_%s@tick_%d_to_%s.mat',...
    motion_name_fr,L_fr_check,motion_name_to);
l = load(tween_path); secs_tween = l.secs_tween; q_revs_tween = l.q_revs_tween; 
L_tween = size(q_revs_tween,1);

% (T_roots_fr,q_revs_fr),L_fr -> (q_revs_tween),L_tween -> (T_roots_to,q_revs_to),L_to
secs_fr = 0:dt:(L_fr-1)*dt;
secs_fr_check = secs_fr(1:L_fr_check);
q_revs_fr_check = q_revs_fr(1:L_fr_check,:);
secs_tween = max(secs_fr_check):dt:(L_tween-1)*dt+max(secs_fr_check);
secs_to = max(secs_tween):dt:(L_to-1)*dt+max(secs_tween);

ca;
% Plot in-betweening motions
axes_info = [0.05,0.12,0.93,0.8];
fig_idx = 1; USE_DRAGZOOM = 1; afs = 15;
set_fig(figure(fig_idx),'pos',[0.0,0.6,0.5,0.35],'AXIS_EQUAL',0,'USE_DRAGZOOM',USE_DRAGZOOM,...
    'axes_info',axes_info,'ax_str','t [sec]','ay_str','x(t)','afs',afs);
plot(secs_fr_check,q_revs_fr_check,'-','color','k');
plot(secs_tween,q_revs_tween,'-','color','r');
plot(secs_to,q_revs_to,'-','color','b');

%%