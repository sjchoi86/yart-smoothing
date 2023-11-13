function [pos,vel,acc,jerk] = check_traj(secs,traj,varargin)
%
% Check trajectory
%

% Parse options
iP = inputParser;
addParameter(iP,'pos_lower',[]); % position upper bound
addParameter(iP,'pos_upper',[]);  % position lower bound
addParameter(iP,'vel_limit',[]);  % velocity limit (upper bound & lower bound)
addParameter(iP,'acc_limit',[]);  % acceleration limit (upper bound & lower bound)
addParameter(iP,'jerk_limit',[]); % jerk limit (upper bound & lower bound)
addParameter(iP,'pos_init',[]);    % initial position
addParameter(iP,'pos_final',[]);   % final position
addParameter(iP,'vel_init',[]);    % initial velocity
addParameter(iP,'vel_final',[]);   % final velocity
addParameter(iP,'acc_init',[]);    % initial acceleration
addParameter(iP,'acc_final',[]);    % final acceleration
addParameter(iP,'fade_dur',[]);    % fade in and out duration in seconds
addParameter(iP,'fade_jerk_limit',[]); % fade in and out jerk limit
addParameter(iP,'VERBOSE',false);
addParameter(iP,'PLOT_FIGS',true);
addParameter(iP,'lw',2);
addParameter(iP,'afs',15);
addParameter(iP,'tfs',15);

addParameter(iP,'fig_idx_offset',0);
addParameter(iP,'fig_pos_yoffset',0);
addParameter(iP,'traj_name',[]);

parse(iP,varargin{:});
pos_lower  = iP.Results.pos_lower;
pos_upper  = iP.Results.pos_upper;
vel_limit  = iP.Results.vel_limit;
acc_limit  = iP.Results.acc_limit;
jerk_limit = iP.Results.jerk_limit;
pos_init   = iP.Results.pos_init;
pos_final  = iP.Results.pos_final;
vel_init   = iP.Results.vel_init;
vel_final  = iP.Results.vel_final;
acc_init   = iP.Results.acc_init;
acc_final  = iP.Results.acc_final;
fade_dur   = iP.Results.fade_dur;
fade_jerk_limit = iP.Results.fade_jerk_limit;
VERBOSE    = iP.Results.VERBOSE;
PLOT_FIGS  = iP.Results.PLOT_FIGS;
lw         = iP.Results.lw;
afs        = iP.Results.afs;
tfs        = iP.Results.tfs;

fig_idx_offset  = iP.Results.fig_idx_offset;
fig_pos_yoffset = iP.Results.fig_pos_yoffset;
traj_name       = iP.Results.traj_name;

% Get velocities and accelerations
L = size(traj,1);
dt = (secs(end)-secs(1))/length(secs); HZ = round(1/dt);
[A_vel,A_acc,A_jerk] = get_A_vel_acc_jerk(L,dt);
pos = traj; vel = A_vel*traj; acc = A_acc*traj; jerk = A_jerk*traj;

% Check
if VERBOSE
    if isempty(traj_name)
        fprintf("[check_traj] \n");
    else
        fprintf("[check_traj] traj_name:[%s] \n",traj_name);
    end
    % First check inequality constraints
    min_pos = min(pos(:));
    max_pos = max(pos(:));
    min_vel = min(vel(:));
    max_vel = max(vel(:));
    min_acc = min(acc(:));
    max_acc = max(acc(:));
    min_jerk = min(jerk(:));
    max_jerk = max(jerk(:));

    max_abs_vel = max(abs(vel(:)));
    max_abs_acc = max(abs(acc(:)));
    max_abs_jerk = max(abs(jerk(:)));

    fprintf(" Position [%.3f]~[%.3f]\n",min_pos,max_pos);
    fprintf(" Velocity [%.3f]~[%.3f]\n",min_vel,max_vel);
    fprintf(" Acceleration [%.3f]~[%.3f]\n",min_acc,max_acc);
    fprintf(" Jerk [%.3f]~[%.3f]\n",min_jerk,max_jerk);

    fade_tick = round(fade_dur*HZ);
    jerk_fade_in = jerk(1:fade_tick,:);
    jerk_fade_out = jerk(end-fade_tick+1:end,:);
    jerk_fade_in_and_out = [jerk_fade_in; jerk_fade_out];
    max_fade_abs_jerk = max(abs(jerk_fade_in_and_out(:)));

    if ~isempty(pos_lower) || ~isempty(pos_upper) || ~isempty(vel_limit) || ~isempty(acc_limit) ...
            || ~isempty(jerk_limit) || ~isempty(fade_jerk_limit)
        fprintf("Inequality constraints:\n");
    end
    if ~isempty(pos_lower)
        fprintf(" min_pos:[%.2f] > pos_lower:[%.2f] ",min_pos,pos_lower);
        if pos_lower < min_pos, fprintf("satisfied.\n");
        else, fprintf(2,"viloated.\n");
        end
    end
    if ~isempty(pos_upper)
        fprintf(" max_pos:[%.2f] < pos_upper:[%.2f] ",max_pos,pos_upper);
        if max_pos < pos_upper, fprintf("satisfied.\n");
        else, fprintf(2,"viloated.\n");
        end
    end
    if ~isempty(vel_limit)
        fprintf(" max_abs_vel:[%.2f] < vel_limit:[%.2f] ",max_abs_vel,vel_limit);
        if max_abs_vel < vel_limit, fprintf("satisfied.\n");
        else, fprintf(2,"viloated.\n");
        end
    end
    if ~isempty(acc_limit)
        fprintf(" max_abs_acc:[%.2f] < acc_limit:[%.2f] ",max_abs_acc,acc_limit);
        if max_abs_acc < acc_limit, fprintf("satisfied.\n");
        else, fprintf(2,"viloated.\n");
        end
    end
    if ~isempty(jerk_limit)
        fprintf(" max_abs_jerk:[%.2f] < jerk_limit:[%.2f] ",max_abs_jerk,jerk_limit);
        if max_abs_jerk < jerk_limit, fprintf("satisfied.\n");
        else, fprintf(2,"viloated.\n");
        end
    end
    if ~isempty(fade_jerk_limit)
        fprintf(" max_fade_abs_jerk:[%.2f] < fade_jerk_limit:[%.2f] ",...
            max_fade_abs_jerk,fade_jerk_limit);
        if max_fade_abs_jerk < fade_jerk_limit, fprintf("satisfied.\n");
        else, fprintf(2,"viloated.\n");
        end
    end

    % Then, check equality constraints
    if ~isempty(pos_init) || ~isempty(pos_final) || ~isempty(vel_init) || ~isempty(vel_final) ...
            || ~isempty(acc_init) || ~isempty(acc_final)
        fprintf("Equality constraints:\n");
    end
    pos_curr_init = pos(1,:);
    if ~isempty(pos_init)
        fprintf(" pos_curr_init:[%.3f] == pos_init:[%.3f] ",...
            pos_curr_init,pos_init);
        if abs(pos_curr_init-pos_init) < 1e-3, fprintf("satisfied.\n");
        else, fprintf(2,"viloated.\n");
        end
    end
    pos_curr_final = pos(end,:);
    if ~isempty(pos_final)
        fprintf(" pos_curr_final:[%.3f] == pos_final:[%.3f] ",...
            pos_curr_final,pos_final);
        if abs(pos_curr_final-pos_final) < 1e-3, fprintf("satisfied.\n");
        else, fprintf(2,"viloated.\n");
        end
    end

    vel_curr_init = vel(1,:);
    if ~isempty(vel_init)
        fprintf(" vel_curr_init:[%.3f] == vel_init:[%.3f] ",...
            vel_curr_init,vel_init);
        if abs(vel_curr_init-vel_init) < 1e-3, fprintf("satisfied.\n");
        else, fprintf(2,"viloated.\n");
        end
    end
    vel_curr_final = vel(end,:);
    if ~isempty(vel_final)
        fprintf(" vel_curr_final:[%.3f] == vel_final:[%.3f] ",...
            vel_curr_final,vel_final);
        if abs(vel_curr_final-vel_final) < 1e-3, fprintf("satisfied.\n");
        else, fprintf(2,"viloated.\n");
        end
    end

    acc_curr_init = acc(1,:);
    if ~isempty(acc_init)
        fprintf(" acc_curr_init:[%.3f] == acc_init:[%.3f] ",...
            acc_curr_init,acc_init);
        if abs(acc_curr_init-acc_init) < 1e-3, fprintf("satisfied.\n");
        else, fprintf(2,"viloated.\n");
        end
    end

    acc_curr_final = acc(end,:);
    if ~isempty(acc_final)
        fprintf(" acc_curr_final:[%.3f] == acc_final:[%.3f] ",...
            acc_curr_final,acc_final);
        if abs(acc_curr_final-acc_final) < 1e-3, fprintf("satisfied.\n");
        else, fprintf(2,"viloated.\n");
        end
    end

end

if PLOT_FIGS
    % Plot original trajectory
    axes_info = [0.12,0.12,0.83,0.8];
    fig_idx = 1+fig_idx_offset; USE_DRAGZOOM = 1;
    set_fig(figure(fig_idx),'pos',[0.0,0.65-fig_pos_yoffset,0.25,0.25],...
        'AXIS_EQUAL',0,'USE_DRAGZOOM',USE_DRAGZOOM,...
        'axes_info',axes_info,'ax_str','t [sec]','ay_str','x(t)','afs',afs);
    if ~isempty(pos_lower)
        plot([secs(1),secs(end)],[pos_lower,pos_lower],'--','Color','b','lineWidth',lw);
    end
    if ~isempty(pos_upper)
        plot([secs(1),secs(end)],[pos_upper,pos_upper],'--','Color','b','lineWidth',lw);
    end
    if ~isempty(pos_init)
        plot(secs(1),pos_init,'o','Color','b','LineWidth',lw,'MarkerSize',10);
    end
    if ~isempty(pos_final)
        plot(secs(end),pos_final,'o','Color','b','LineWidth',lw,'MarkerSize',10);
    end
    plot(secs,traj,'LineStyle','-','Color','k','LineWidth',lw);
    xlim([secs(1),secs(end)]);
    if isempty(traj_name)
        plot_title('Original Trajectories','fig_idx',fig_idx,'tfs',tfs);
    else
        plot_title(['Original Trajectories of [',traj_name,']'],'fig_idx',fig_idx,'tfs',tfs);
    end

    % Plot velocity
    fig_idx = 2+fig_idx_offset;
    set_fig(figure(fig_idx),'pos',[0.25,0.65-fig_pos_yoffset,0.25,0.25],...
        'AXIS_EQUAL',0,'USE_DRAGZOOM',USE_DRAGZOOM,...
        'axes_info',axes_info,'ax_str','t [sec]','ay_str','$\dot{x}(t)$','afs',afs);
    if ~isempty(vel_limit)
        plot([secs(1),secs(end)],[-vel_limit,-vel_limit],'--','Color','b','lineWidth',lw);
        plot([secs(1),secs(end)],[vel_limit,vel_limit],'--','Color','b','lineWidth',lw);
    end
    if ~isempty(vel_init)
        plot(secs(1),vel_init,'o','Color','b','LineWidth',lw,'MarkerSize',10);
    end
    if ~isempty(vel_final)
        plot(secs(end),vel_final,'o','Color','b','LineWidth',lw,'MarkerSize',10);
    end
    plot(secs,vel,'LineStyle','-','Color','k','LineWidth',lw);
    xlim([secs(1),secs(end)]);
    plot_title('Numerical Velocity','fig_idx',fig_idx,'tfs',tfs);

    % Plot acceleration
    fig_idx = 3+fig_idx_offset;
    set_fig(figure(fig_idx),...
        'pos',[0.5,0.65-fig_pos_yoffset,0.25,0.25],...
        'AXIS_EQUAL',0,'USE_DRAGZOOM',USE_DRAGZOOM,...
        'axes_info',axes_info,'ax_str','t [sec]','ay_str','$\ddot{x}(t)$','afs',afs);
    if ~isempty(acc_limit)
        plot([secs(1),secs(end)],[-acc_limit,-acc_limit],'--','Color','b','lineWidth',lw);
        plot([secs(1),secs(end)],[acc_limit,acc_limit],'--','Color','b','lineWidth',lw);
    end
    if ~isempty(acc_init)
        plot(secs(1),acc_init,'o','Color','b','LineWidth',lw,'MarkerSize',10);
    end
    if ~isempty(acc_final)
        plot(secs(end),acc_final,'o','Color','b','LineWidth',lw,'MarkerSize',10);
    end
    plot(secs,acc,'LineStyle','-','Color','k','LineWidth',lw);
    xlim([secs(1),secs(end)]);
    plot_title('Numerical Acceleration','fig_idx',fig_idx,'tfs',tfs);

    % Plot jerk
    fig_idx = 4+fig_idx_offset;
    set_fig(figure(fig_idx),...
        'pos',[0.75,0.65-fig_pos_yoffset,0.25,0.25],...
        'AXIS_EQUAL',0,'USE_DRAGZOOM',USE_DRAGZOOM,...
        'axes_info',axes_info,'ax_str','t [sec]','ay_str','jerk','afs',afs);
    plot(secs,jerk,'LineStyle','-','Color','k','LineWidth',lw);
    xlim([secs(1),secs(end)]);
    plot_title('Numerical Jerk','fig_idx',fig_idx,'tfs',tfs);
    if ~isempty(jerk_limit)
        plot([secs(1),secs(end)],[-jerk_limit,-jerk_limit],'--','Color','b','lineWidth',lw);
        plot([secs(1),secs(end)],[jerk_limit,jerk_limit],'--','Color','b','lineWidth',lw);
    end
    drawnow;
end
