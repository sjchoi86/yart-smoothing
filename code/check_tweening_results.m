function check_tweening_results(...
    secs_fr,traj_fr,secs_tween,traj_tween,secs_to,traj_to,varargin)
%
% Check tweening results
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
addParameter(iP,'PLOT_FADE',true);
addParameter(iP,'USE_DRAGZOOM',true);
addParameter(iP,'VERBOSE',true);
addParameter(iP,'lw',2);
addParameter(iP,'afs',15);
addParameter(iP,'tfs',15);
parse(iP,varargin{:});
pos_lower    = iP.Results.pos_lower;
pos_upper    = iP.Results.pos_upper;
vel_limit    = iP.Results.vel_limit;
acc_limit    = iP.Results.acc_limit;
jerk_limit   = iP.Results.jerk_limit;
pos_init     = iP.Results.pos_init;
pos_final    = iP.Results.pos_final;
vel_init     = iP.Results.vel_init;
vel_final    = iP.Results.vel_final;
acc_init     = iP.Results.acc_init;
acc_final    = iP.Results.acc_final;
USE_DRAGZOOM = iP.Results.USE_DRAGZOOM;
VERBOSE      = iP.Results.VERBOSE;
lw           = iP.Results.lw;
afs          = iP.Results.afs;
tfs          = iP.Results.tfs;

% Constants
secs_fr_hat    = secs_fr;
secs_tween_hat = secs_tween + max(secs_fr_hat);
secs_to_hat    = secs_to + max(secs_tween_hat);

% Copute vel, acc, jerk 
[vel_fr,acc_fr,jerk_fr]          = get_vel_acc_jerk(secs_fr,traj_fr);
[vel_tween,acc_tween,jerk_tween] = get_vel_acc_jerk(secs_tween,traj_tween);
[vel_to,acc_to,jerk_to]          = get_vel_acc_jerk(secs_to,traj_to);

% Plot position
axes_info = [0.10,0.12,0.88,0.8];
fig_idx = 1;
set_fig(figure(fig_idx),'pos',[0.0,0.65,0.25,0.25],'AXIS_EQUAL',0,'USE_DRAGZOOM',USE_DRAGZOOM,...
    'axes_info',axes_info,'ax_str','t [sec]','ay_str','x(t)','afs',afs);
if ~isempty(pos_lower)
    plot([secs_tween_hat(1),secs_tween_hat(end)],[pos_lower,pos_lower],...
        '--','Color','m','lineWidth',lw);
end
if ~isempty(pos_upper)
    plot([secs_tween_hat(1),secs_tween_hat(end)],[pos_upper,pos_upper],...
        '--','Color','m','lineWidth',lw);
end
if ~isempty(pos_init)
    plot(secs_tween_hat(1),pos_init,'o','Color','m','LineWidth',lw,'MarkerSize',10);
end
if ~isempty(pos_final)
    plot(secs_tween_hat(end),pos_final,'o','Color','m','LineWidth',lw,'MarkerSize',10);
end
plot(secs_fr_hat,traj_fr,'LineStyle','-','Color','k','LineWidth',lw);
plot(secs_tween_hat,traj_tween,'LineStyle','-','Color','r','LineWidth',lw);
plot(secs_to_hat,traj_to,'LineStyle','-','Color','b','LineWidth',lw);
plot_title('Original Trajectory','fig_idx',fig_idx,'tfs',tfs);

% Plot velocity
fig_idx = 2;
set_fig(figure(fig_idx),'pos',[0.25,0.65,0.25,0.25],'AXIS_EQUAL',0,'USE_DRAGZOOM',USE_DRAGZOOM,...
    'axes_info',axes_info,'ax_str','t [sec]','ay_str','$\dot{x}(t)$','afs',afs);
if ~isempty(vel_limit)
    plot([secs_tween_hat(1),secs_tween_hat(end)],[-vel_limit,-vel_limit],...
        '--','Color','b','lineWidth',lw);
    plot([secs_tween_hat(1),secs_tween_hat(end)],[vel_limit,vel_limit],...
        '--','Color','b','lineWidth',lw);
end
if ~isempty(vel_init)
    plot(secs_tween_hat(1),vel_init,'o','Color','m','LineWidth',lw,'MarkerSize',10);
end
if ~isempty(vel_final)
    plot(secs_tween_hat(end),vel_final,'o','Color','m','LineWidth',lw,'MarkerSize',10);
end
plot(secs_fr_hat,vel_fr,'LineStyle','-','Color','k','LineWidth',lw);
plot(secs_tween_hat,vel_tween,'LineStyle','-','Color','r','LineWidth',lw);
plot(secs_to_hat,vel_to,'LineStyle','-','Color','b','LineWidth',lw);
plot_title('Numerical Velocity','fig_idx',fig_idx,'tfs',tfs);

% Plot acceleration
fig_idx = 3;
set_fig(figure(fig_idx),'pos',[0.5,0.65,0.25,0.25],'AXIS_EQUAL',0,'USE_DRAGZOOM',USE_DRAGZOOM,...
    'axes_info',axes_info,'ax_str','t [sec]','ay_str','$\ddot{x}(t)$','afs',afs);
if ~isempty(acc_limit)
    plot([secs_tween_hat(1),secs_tween_hat(end)],[-acc_limit,-acc_limit],...
        '--','Color','b','lineWidth',lw);
    plot([secs_tween_hat(1),secs_tween_hat(end)],[acc_limit,acc_limit],...
        '--','Color','b','lineWidth',lw);
end
if ~isempty(acc_init)
    plot(secs_tween_hat(1),acc_init,'o','Color','m','LineWidth',lw,'MarkerSize',10);
end
if ~isempty(acc_final)
    plot(secs_tween_hat(end),acc_final,'o','Color','m','LineWidth',lw,'MarkerSize',10);
end
plot(secs_fr_hat,acc_fr,'LineStyle','-','Color','k','LineWidth',lw);
plot(secs_tween_hat,acc_tween,'LineStyle','-','Color','r','LineWidth',lw);
plot(secs_to_hat,acc_to,'LineStyle','-','Color','b','LineWidth',lw);
plot_title('Numerical Acceleration','fig_idx',fig_idx,'tfs',tfs);

% Plot jerk
fig_idx = 4;
set_fig(figure(fig_idx),'pos',[0.75,0.65,0.25,0.25],'AXIS_EQUAL',0,'USE_DRAGZOOM',USE_DRAGZOOM,...
    'axes_info',axes_info,'ax_str','t [sec]','ay_str','jerk(t)','afs',afs);
if ~isempty(jerk_limit)
    plot([secs_tween_hat(1),secs_tween_hat(end)],[-jerk_limit,-jerk_limit],...
        '--','Color','b','lineWidth',lw);
    plot([secs_tween_hat(1),secs_tween_hat(end)],[jerk_limit,jerk_limit],...
        '--','Color','b','lineWidth',lw);
end
plot(secs_fr_hat,jerk_fr,'LineStyle','-','Color','k','LineWidth',lw);
plot(secs_tween_hat,jerk_tween,'LineStyle','-','Color','r','LineWidth',lw);
plot(secs_to_hat,jerk_to,'LineStyle','-','Color','b','LineWidth',lw);
plot_title('Numerical Jerk','fig_idx',fig_idx,'tfs',tfs);