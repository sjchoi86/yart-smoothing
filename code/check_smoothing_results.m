function [pos,vel,acc,jerk,pos_smt,vel_smt,acc_smt,jerk_smt] = ...
    check_smoothing_results(secs,traj,traj_smt,varargin)
%
% Examine smoothing results
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
addParameter(iP,'PLOT_FADE',true);
addParameter(iP,'VERBOSE',true);
addParameter(iP,'lw',2);
addParameter(iP,'afs',15);
addParameter(iP,'tfs',15);
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
PLOT_FADE  = iP.Results.PLOT_FADE;
VERBOSE    = iP.Results.VERBOSE;
lw         = iP.Results.lw;
afs        = iP.Results.afs;
tfs        = iP.Results.tfs;

% Get velocities and accelerations
L = size(traj,1); dim = size(traj,2);
dt = (secs(end)-secs(1))/length(secs); HZ = round(1/dt);
[A_vel,A_acc,A_jerk] = get_A_vel_acc_jerk(L,dt);
pos = traj; vel = A_vel*traj; acc = A_acc*traj; jerk = A_jerk*traj;
pos_smt = traj_smt; vel_smt = A_vel*traj_smt; acc_smt = A_acc*traj_smt;
jerk_smt = A_jerk*traj_smt;

% Check
if VERBOSE

    for d_idx = 1:dim % for each dimension
        
        if dim > 1
            fprintf("\nExamining [%d/%d]-th dimension. \n",d_idx,dim);
        end
        % First check inequality constraints
        min_pos = min(pos_smt(:,d_idx));
        max_pos = max(pos_smt(:,d_idx));
        max_abs_vel = max(abs(vel_smt(:,d_idx)));
        max_abs_acc = max(abs(acc_smt(:,d_idx)));
        max_abs_jerk = max(abs(jerk_smt(:,d_idx)));

        fade_tick = round(fade_dur*HZ);
        jerk_smt_fade_in = jerk_smt(1:fade_tick,d_idx);
        jerk_smt_fade_out = jerk_smt(end-fade_tick+1:end,d_idx);
        jerk_smt_fade_in_and_out = [jerk_smt_fade_in; jerk_smt_fade_out];
        max_fade_abs_jerk = max(abs(jerk_smt_fade_in_and_out(:)));

        fprintf("[check_smoothing_results] \n");
        fprintf("Inequality constraints:\n");
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
        fprintf("Equality constraints:\n");
        pos_smt_init = pos_smt(1,d_idx);
        if ~isempty(pos_init)
            fprintf(" pos_smt_init:[%.3f] == pos_init:[%.3f] ",...
                pos_smt_init,pos_init);
            if abs(pos_smt_init-pos_init) < 1e-3, fprintf("satisfied.\n");
            else, fprintf(2,"viloated.\n");
            end
        end
        pos_smt_final = pos_smt(end,d_idx);
        if ~isempty(pos_final)
            fprintf(" pos_smt_final:[%.3f] == pos_final:[%.3f] ",...
                pos_smt_final,pos_final);
            if abs(pos_smt_final-pos_final) < 1e-3, fprintf("satisfied.\n");
            else, fprintf(2,"viloated.\n");
            end
        end
        vel_smt_init = vel_smt(1,d_idx);
        if ~isempty(vel_init)
            fprintf(" vel_smt_init:[%.3f] == vel_init:[%.3f] ",...
                vel_smt_init,vel_init);
            if abs(vel_smt_init-vel_init) < 1e-3, fprintf("satisfied.\n");
            else, fprintf(2,"viloated.\n");
            end
        end
        vel_smt_final = vel_smt(end,d_idx);
        if ~isempty(vel_final)
            fprintf(" vel_smt_final:[%.3f] == vel_final:[%.3f] ",...
                vel_smt_final,vel_final);
            if abs(vel_smt_final-vel_final) < 1e-3, fprintf("satisfied.\n");
            else, fprintf(2,"viloated.\n");
            end
        end
        acc_smt_init = acc_smt(1,d_idx);
        if ~isempty(acc_init)
            fprintf(" acc_smt_init:[%.3f] == acc_init:[%.3f] ",...
                acc_smt_init,acc_init);
            if abs(acc_smt_init-acc_init) < 1e-3, fprintf("satisfied.\n");
            else, fprintf(2,"viloated.\n");
            end
        end
        acc_smt_final = acc_smt(end,d_idx);
        if ~isempty(acc_final)
            fprintf(" acc_smt_final:[%.3f] == acc_final:[%.3f] ",...
                acc_smt_final,acc_final);
            if abs(acc_smt_final-acc_final) < 1e-3, fprintf("satisfied.\n");
            else, fprintf(2,"viloated.\n");
            end
        end
    
    end % for d_idx = 1:dim % for each dimension

end


% Plot original trajectory
axes_info = [0.12,0.12,0.83,0.8];
fig_idx = 1; USE_DRAGZOOM = 1;
set_fig(figure(fig_idx),'pos',[0.0,0.65,0.25,0.25],'AXIS_EQUAL',0,'USE_DRAGZOOM',USE_DRAGZOOM,...
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
plot(secs,traj_smt,'LineStyle','--','Color','r','LineWidth',lw);
xlim([secs(1),secs(end)]);
plot_title('Original (Black) and Smoothed (Red) Trajectories','fig_idx',fig_idx,'tfs',tfs);

% Plot velocity
fig_idx = 2;
set_fig(figure(fig_idx),'pos',[0.25,0.65,0.25,0.25],'AXIS_EQUAL',0,'USE_DRAGZOOM',USE_DRAGZOOM,...
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
plot(secs,vel_smt,'LineStyle','--','Color','r','LineWidth',lw);
xlim([secs(1),secs(end)]);
plot_title('Numerical Velocity','fig_idx',fig_idx,'tfs',tfs);

% Plot acceleration
fig_idx = 3;
set_fig(figure(fig_idx),'pos',[0.5,0.65,0.25,0.25],'AXIS_EQUAL',0,'USE_DRAGZOOM',USE_DRAGZOOM,...
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
plot(secs,acc_smt,'LineStyle','--','Color','r','LineWidth',lw);
xlim([secs(1),secs(end)]);
plot_title('Numerical Acceleration','fig_idx',fig_idx,'tfs',tfs);

% Plot jerk
fig_idx = 4;
set_fig(figure(fig_idx),'pos',[0.75,0.65,0.25,0.25],'AXIS_EQUAL',0,'USE_DRAGZOOM',USE_DRAGZOOM,...
    'axes_info',axes_info,'ax_str','t [sec]','ay_str','jerk','afs',afs);
if ~isempty(jerk_limit)
    plot([secs(1),secs(end)],[-jerk_limit,-jerk_limit],'--','Color','b','lineWidth',lw);
    plot([secs(1),secs(end)],[jerk_limit,jerk_limit],'--','Color','b','lineWidth',lw);
end
if ~isempty(fade_dur)
    plot([secs(1),secs(1)+fade_dur],[fade_jerk_limit,fade_jerk_limit],...
        '--','Color','b','lineWidth',lw);
    plot([secs(1),secs(1)+fade_dur],[-fade_jerk_limit,-fade_jerk_limit],...
        '--','Color','b','lineWidth',lw);
    plot([secs(end)-fade_dur,secs(end)],[fade_jerk_limit,fade_jerk_limit],...
        '--','Color','b','lineWidth',lw);
    plot([secs(end)-fade_dur,secs(end)],[-fade_jerk_limit,-fade_jerk_limit],...
        '--','Color','b','lineWidth',lw);
end
plot(secs,jerk,'LineStyle','-','Color','k','LineWidth',lw);
plot(secs,jerk_smt,'LineStyle','--','Color','r','LineWidth',lw);
xlim([secs(1),secs(end)]);
plot_title('Numerical Jerk','fig_idx',fig_idx,'tfs',tfs);

% Plot fade-in trajectory
if PLOT_FADE && ~isempty(fade_dur)

    min_traj = min(min(min(traj),min(traj_smt)));
    max_traj = max(max(max(traj),max(traj_smt)));

    fig_idx = 5; USE_DRAGZOOM = 1;
    set_fig(figure(fig_idx),...
        'pos',[0.0,0.4,0.25,0.25],'AXIS_EQUAL',0,'USE_DRAGZOOM',USE_DRAGZOOM,...
        'axes_info',axes_info,'ax_str','t [sec]','ay_str','x(t)','afs',afs);
    h_fill = fill(...
        [secs(1),secs(1)+fade_dur,secs(1)+fade_dur,secs(1)],...
        [min_traj,min_traj,max_traj,max_traj],...
        'm','LineStyle',':');
    set(h_fill,'FaceAlpha',0.2);
    plot(secs,traj,'LineStyle','-','Color','k','LineWidth',lw);
    plot(secs,traj_smt,'LineStyle','--','Color','r','LineWidth',lw);
    plot([secs(1)+fade_dur,secs(1)+fade_dur],1000*[-1,+1],'r--');
    plot_title('Fade-in of Original (Black) and Smoothed (Red) Trajectories',...
        'fig_idx',fig_idx,'tfs',tfs);
    xlim([secs(1),secs(1)+fade_dur*3]);
    ylim([min_traj,max_traj]);

    fig_idx = 6; USE_DRAGZOOM = 1;
    set_fig(figure(fig_idx),...
        'pos',[0.25,0.4,0.25,0.25],'AXIS_EQUAL',0,'USE_DRAGZOOM',USE_DRAGZOOM,...
        'axes_info',axes_info,'ax_str','t [sec]','ay_str','x(t)','afs',afs);
    h_fill = fill(...
        [secs(end)-fade_dur,secs(end),secs(end),secs(end)-fade_dur],...
        [min_traj,min_traj,max_traj,max_traj],...
        'm','LineStyle',':');
    set(h_fill,'FaceAlpha',0.2);
    plot(secs,traj,'LineStyle','-','Color','k','LineWidth',lw);
    plot(secs,traj_smt,'LineStyle','--','Color','r','LineWidth',lw);
    plot([secs(end)-fade_dur,secs(end)-fade_dur],1000*[-1,+1],'r--');
    plot_title('Fade-out of Original (Black) and Smoothed (Red) Trajectories',...
        'fig_idx',fig_idx,'tfs',tfs);
    xlim([secs(end)-fade_dur*3,secs(end)]);
    ylim([min_traj,max_traj]);
end

drawnow;
