addpath_yart
%%
ccc
% Load motions
motion_idx = 1;
[secs,q_revs,T_roots,chain_robot] = load_motion(motion_idx);
HZ = round(1/(secs(2)-secs(1)));

L = size(secs,1); % motion length
chains = cell(1,L);
for tick = 1:L
    q = q_revs(tick,:);
    T_root = T_roots{tick};
    chain_robot = update_chain_q_root_T(chain_robot,q,T_root);
    chain_robot_ground = move_chain_two_feet_on_ground(chain_robot);
    chains{tick} = chain_robot_ground; % append chain
end
axis_info = get_axis_info_from_chains(chains,'margin',0.5);

% Animate
vid_path = '../vid/playback_motion.mp4';
vobj = init_vid_record(vid_path,'HZ',HZ);
for tick = 1:L
    sec = secs(tick);
    chain_robot_ground = chains{tick};

    % Check SC
    SC = check_sc(chain_robot_ground);
    if SC
        fprintf(2,"[%d/%d] self-collision occurred. \n",tick,L);
    end

    % Plot
    fig_idx = 1;
    fig = plot_chain(chain_robot_ground,'fig_idx',1,'fig_pos',[0.0,0.5,0.2,0.45],...
        'PLOT_ROTATE_AXIS',0,'PLOT_CAPSULE',1,...
        'axis_info',axis_info);
    title_str = sprintf("[%d/%d] SC:[%d]",tick,L,SC);
    plot_title(title_str,'fig_idx',fig_idx);
    drawnow; pause_invalid_handle(fig);
    record_vid(vobj,'fig',fig);
end
end_vid_record(vobj);

%%