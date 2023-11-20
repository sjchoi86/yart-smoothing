function [secs_tween,traj_tween,exit_flag] = optimization_based_tweening_1d(...
    secs_fr,traj_fr,secs_to,traj_to,dur_tweening,varargin)
%
% Optimization-based tweening of two trajectories
%

% Parse options
iP = inputParser;
addParameter(iP,'pos_lower',[]); % position upper bound
addParameter(iP,'pos_upper',[]);  % position lower bound

addParameter(iP,'vel_limit',[]);  % velocity limit (upper bound & lower bound)
addParameter(iP,'acc_limit',[]);  % acceleration limit (upper bound & lower bound)
addParameter(iP,'jerk_limit',[]); % jerk limit (upper bound & lower bound)

addParameter(iP,'pos_init',[]);
addParameter(iP,'pos_final',[]);
addParameter(iP,'vel_init',[]);
addParameter(iP,'vel_final',[]);
addParameter(iP,'acc_init',[]);
addParameter(iP,'acc_final',[]);

addParameter(iP,'max_iter',10000); 
addParameter(iP,'optm_tol',1e-6); 
addParameter(iP,'step_tol',1e-6); 
addParameter(iP,'VERBOSE',true); 
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

max_iter   = iP.Results.max_iter;
optm_tol   = iP.Results.optm_tol;
step_tol   = iP.Results.step_tol;
VERBOSE    = iP.Results.VERBOSE;

% Constants
dt = secs_fr(2)-secs_fr(1);
HZ = 1/dt;

% from trajectory
[vel_fr,acc_fr,jerk_fr] = get_vel_acc_jerk(secs_fr,traj_fr);

% to trajectory
[vel_to,acc_to,jerk_to] = get_vel_acc_jerk(secs_to,traj_to);

% Default equality constraints
% pos_init   = traj_fr(end);
% pos_final  = traj_to(1);
% vel_init   = vel_fr(end);
% vel_final  = vel_to(1);
% acc_init   = acc_fr(end);
% acc_final  = acc_to(1);

% tweening trajectory
L_tween = round(dur_tweening*HZ);
secs_tween = linspace(0,dur_tweening,L_tween)';
[A_vel,A_acc,A_jerk] = get_A_vel_acc_jerk(L_tween,dt);

% Objective function
fun = @(x)( norm(A_vel*x,2)/dur_tweening ); % minimum distance objective
traj_interp = linspace(traj_fr(end),traj_to(1),L_tween)';
x_init = traj_interp; % initial solution

% Run optimization
opt = optimoptions('fmincon', ...
    'OptimalityTolerance', optm_tol, ...
    'StepTolerance', step_tol, ...
    'MaxFunctionEvaluations', max_iter,...
    'MaxIterations', max_iter, ...
    'Algorithm','interior-point', ...
    'Display', 'off');

% Inequality constraints
A_ineq = []; b_ineq = [];
E = eye(L_tween,L_tween);
if ~isempty(pos_lower) % positional lower bound
    A_ineq = [A_ineq ; -E];
    b_ineq = [b_ineq ; -pos_lower*ones(L_tween,1)];
end
if ~isempty(pos_upper) % positional upper bound
    A_ineq = [A_ineq ; E];
    b_ineq = [b_ineq ; pos_upper*ones(L_tween,1)];
end
if ~isempty(vel_limit) % velocity limit
    A_ineq = [A_ineq ; A_vel ; -A_vel];
    b_ineq = [b_ineq ; vel_limit*ones(L_tween,1) ; vel_limit*ones(L_tween,1)];
end
if ~isempty(acc_limit) % acceleration limit
    A_ineq = [A_ineq ; A_acc ; -A_acc];
    b_ineq = [b_ineq ; acc_limit*ones(L_tween,1) ; acc_limit*ones(L_tween,1)];
end
if ~isempty(jerk_limit) % jerk limit
    A_ineq = [A_ineq ; A_jerk ; -A_jerk];
    b_ineq = [b_ineq ; jerk_limit*ones(L_tween,1) ; jerk_limit*ones(L_tween,1)];
end

% Equality constraint
A_eq = []; b_eq = [];
if ~isempty(pos_init) % initial position 
    a = zeros(1,L_tween); a(1) = 1;
    A_eq = [A_eq; a];
    b_eq = [b_eq; pos_init];
end
if ~isempty(pos_final) % final position 
    a = zeros(1,L_tween); a(end) = 1;
    A_eq = [A_eq; a];
    b_eq = [b_eq; pos_final];
end
if ~isempty(vel_init) % initial velocity 
    A_eq = [A_eq; A_vel(1,:)];
    b_eq = [b_eq; vel_init];
end
if ~isempty(vel_final) % final velocity 
    A_eq = [A_eq; A_vel(end,:)];
    b_eq = [b_eq; vel_final];
end
if ~isempty(acc_init) % initial acceleration 
    A_eq = [A_eq; A_acc(1,:)];
    b_eq = [b_eq; acc_init];
end
if ~isempty(acc_final) % final acceleration 
    A_eq = [A_eq; A_acc(end,:)];
    b_eq = [b_eq; acc_final];
end

% Run optimization
[traj_tween,fval,exit_flag] = fmincon(...
    fun,...
    x_init,...
    A_ineq,b_ineq,...
    A_eq,b_eq,...
    [],[],[],opt);

if VERBOSE
    fprintf("[optimization_based_tweening_1d] fmincon done. fval:[%.3f] \n",fval);
    print_fmincon_exit_flag(exit_flag);
end



