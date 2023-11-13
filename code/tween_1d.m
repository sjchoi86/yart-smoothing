addpath_yart
%% In-betweening 1D trajectories
ccc

HZ = 30;
dt = 1/HZ;
len_param = 0.4;

% First, sample two trajectories from GP prior
rng(4); % fix seed

% trajectory (from)
t_max = 5; L_fr = round(t_max*HZ); secs_fr = linspace(0,t_max,L_fr)';
K = kernel_levse(secs_fr,secs_fr,ones(L_fr,1),ones(L_fr,1),[1,len_param]);
traj_fr = chol(K+1e-10*eye(L_fr,L_fr))'*randn(L_fr,1);
[A_vel_fr,~,~] = get_A_vel_acc_jerk(L_fr,dt);
vel_fr = A_vel_fr*traj_fr;

% trajectory (to)
t_max = 3; L_to = round(t_max*HZ); secs_to = linspace(0,t_max,L_to)';
K = kernel_levse(secs_to,secs_to,ones(L_to,1),ones(L_to,1),[1,len_param]);
traj_to = chol(K+1e-10*eye(L_to,L_to))'*randn(L_to,1);
[A_vel_to,~,~] = get_A_vel_acc_jerk(L_to,dt);
vel_to = A_vel_to*traj_to;

% base tweening duration
dur_tweening = 3;

% tweening
L_tween = round(dur_tweening*HZ);
secs_tween = linspace(secs_fr(end),secs_fr(end)+dur_tweening,L_tween)';
traj_interp = linspace(traj_fr(end),traj_to(1),L_tween)';
[A_vel_tween,A_acc_tween,A_jerk_tween] = get_A_vel_acc_jerk(L_to,dt);

% Objective function
fun = @(x)( norm(A_vel_tween*x,2)/dur_tweening ); % minimum distance objective
x0 = traj_interp; % initial solution

x_lower_limit = -2;
x_upper_limit = 3.5;
acc_limit     = 1000;
jerk_limit    = 50;

% Run optimization
max_iter = 10000;
opt = optimoptions('fmincon', ...
    'OptimalityTolerance', 1e-6, ...
    'StepTolerance', 1e-6, ...
    'MaxFunctionEvaluations', max_iter,...
    'MaxIterations', max_iter, ...
    'Algorithm','interior-point', ...
    'Display', 'off');
% Inequality const. 1) acc limit, 2) jerk limit, 3) upper position limit, 4) lower position limit
E = eye(L_tween,L_tween);
A = [...
    A_acc_tween;-A_acc_tween;... % 1) acc limit
    A_jerk_tween;-A_jerk_tween;... % 2) jerk limit
    E;... % 3) upper position limit
    -E... % 4) lower position limit
    ];
b = [...
    acc_limit*ones(L_tween,1);acc_limit*ones(L_tween,1);... % 1) acc limit
    jerk_limit*ones(L_tween,1);jerk_limit*ones(L_tween,1);... % 2) jerk limit
    x_upper_limit*ones(L_tween,1);... % 3) upper position limit
    -x_lower_limit*ones(L_tween,1);... % 4) lower position limit
    ];
% Equality const. 1) start pos, 2) final pos, 3) start vel, 4) final vel
a1 = zeros(1,L_tween); a1(1) = 1;
a2 = zeros(1,L_tween); a2(end) = 1;
A_eq = [....
    a1;... % 1) start pos
    a2;... % 2) final pos
    A_vel_tween(1,:);... % 3) start vel
    A_vel_tween(end,:)... % 4) final vel
    ];
b_eq = [...
    traj_fr(end);... % 1) start pos
    traj_to(1);... % 2) final pos
    vel_fr(end);... % 3) start vel
    vel_to(1)... % 4) final vel
    ];
[traj_hat,fval,exit_flag] = fmincon(fun,x0,...
    A,b,...
    A_eq,b_eq,...
    [],[],[],opt);
fprintf("Optimization done.\n");
print_fmincon_exit_flag(exit_flag);

% Copute vel, acc, jerk of the optimized trajectories
vel_tween_hat = A_vel_tween*traj_hat;
acc_tween_hat = A_acc_tween*traj_hat;
jerk_tween_hat = A_jerk_tween*traj_hat;

% Plot original trajectory
axes_info = [0.10,0.12,0.88,0.8];
fig_idx = 1;
set_fig(figure(fig_idx),'pos',[0.0,0.65,0.25,0.25],'AXIS_EQUAL',0,'USE_DRAGZOOM',0,...
    'axes_info',axes_info,'ax_str','t [sec]','ay_str','x(t)','afs',18);
plot(secs_fr,traj_fr,'LineStyle','-','Color','k','LineWidth',2);
% plot(secs_tween,traj_interp,'LineStyle','-','Color','r','LineWidth',1/2);
plot(secs_tween,traj_hat,'LineStyle','-','Color','r','LineWidth',2);
plot(secs_fr(end)+secs_to+dur_tweening,traj_to,'LineStyle','-','Color','b','LineWidth',2);
plot_title('Original Trajectory','fig_idx',fig_idx,'tfs',18);

% Plot velocity
fig_idx = 2;
set_fig(figure(fig_idx),'pos',[0.25,0.65,0.25,0.25],'AXIS_EQUAL',0,'USE_DRAGZOOM',0,...
    'axes_info',axes_info,'ax_str','t [sec]','ay_str','$\dot{x}(t)$','afs',18);
plot(secs_tween,vel_tween_hat,'LineStyle','-','Color','k','LineWidth',2);
plot_title('Numerical Velocity','fig_idx',fig_idx,'tfs',18);

% Plot acceleration
fig_idx = 3;
set_fig(figure(fig_idx),'pos',[0.5,0.65,0.25,0.25],'AXIS_EQUAL',0,'USE_DRAGZOOM',0,...
    'axes_info',axes_info,'ax_str','t [sec]','ay_str','$\ddot{x}(t)$','afs',18);
plot(secs_tween,acc_tween_hat,'LineStyle','-','Color','k','LineWidth',2);
plot_title('Numerical Acceleration','fig_idx',fig_idx,'tfs',18);

% Plot jerk
fig_idx = 4;
set_fig(figure(fig_idx),'pos',[0.75,0.65,0.25,0.25],'AXIS_EQUAL',0,'USE_DRAGZOOM',0,...
    'axes_info',axes_info,'ax_str','t [sec]','ay_str','jerk','afs',18);
plot(secs_tween,jerk_tween_hat,'LineStyle','-','Color','k','LineWidth',2);
plot_title('Numerical Jerk','fig_idx',fig_idx,'tfs',18);