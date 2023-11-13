function [traj_smt,exit_flag] = optimization_based_smoothing_1d(secs,traj,varargin)
%
% Run optimization-based smoothing
%
% :param secs: [L x 1]
% :param traj: [L x 1]
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
addParameter(iP,'acc_final',[]);   % final acceleration
addParameter(iP,'fade_dur',[]);    % fade in and out duration in seconds
addParameter(iP,'fade_jerk_limit',[]); % fade in and out jerk limit
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
fade_dur   = iP.Results.fade_dur;
fade_jerk_limit = iP.Results.fade_jerk_limit;
max_iter   = iP.Results.max_iter;
optm_tol   = iP.Results.optm_tol;
step_tol   = iP.Results.step_tol;
VERBOSE    = iP.Results.VERBOSE;

% Constants
L = size(traj,1);
dt = (secs(end)-secs(1))/length(secs);
HZ = round(1/dt);
[A_vel,A_acc,A_jerk] = get_A_vel_acc_jerk(L,dt);

% Objective function
fun = @(x)( norm(traj-x,2) );
x_init = traj; % zeros(L,1); % initial solution

% Constraints
opt = optimoptions('fmincon', ...
    'OptimalityTolerance', optm_tol, ...
    'StepTolerance', step_tol, ...
    'MaxFunctionEvaluations', max_iter,...
    'MaxIterations', max_iter, ...
    'Algorithm','interior-point', ...
    'Display', 'off');

% Inequality constraint
A_ineq = []; b_ineq = [];
E = eye(L,L);
if ~isempty(pos_lower) % positional lower bound
    A_ineq = [A_ineq ; -E];
    b_ineq = [b_ineq ; -pos_lower*ones(L,1)];
end
if ~isempty(pos_upper) % positional upper bound
    A_ineq = [A_ineq ; E];
    b_ineq = [b_ineq ; pos_upper*ones(L,1)];
end
if ~isempty(vel_limit) % velocity limit
    A_ineq = [A_ineq ; A_vel ; -A_vel];
    b_ineq = [b_ineq ; vel_limit*ones(L,1) ; vel_limit*ones(L,1)];
end
if ~isempty(acc_limit) % acceleration limit
    A_ineq = [A_ineq ; A_acc ; -A_acc];
    b_ineq = [b_ineq ; acc_limit*ones(L,1) ; acc_limit*ones(L,1)];
end
if ~isempty(jerk_limit) % jerk limit
    A_ineq = [A_ineq ; A_jerk ; -A_jerk];
    b_ineq = [b_ineq ; jerk_limit*ones(L,1) ; jerk_limit*ones(L,1)];
end

if ~isempty(fade_dur) % jerk limit with 'fade_dur'
    fade_tick = round(fade_dur*HZ);
    A_ineq = [A_ineq ; A_jerk(1:fade_tick,:) ; -A_jerk(1:fade_tick,:)];
    b_ineq = [b_ineq ; fade_jerk_limit*ones(fade_tick,1) ; fade_jerk_limit*ones(fade_tick,1)];
    A_ineq = [A_ineq ; A_jerk(end-fade_tick+1:end,:) ; -A_jerk(end-fade_tick+1:end,:)];
    b_ineq = [b_ineq ; fade_jerk_limit*ones(fade_tick,1) ; fade_jerk_limit*ones(fade_tick,1)];
end

% Equality constraint
A_eq = []; b_eq = [];
if ~isempty(pos_init) % initial position 
    a = zeros(1,L); a(1) = 1;
    A_eq = [A_eq; a];
    b_eq = [b_eq; pos_init];
end
if ~isempty(pos_final) % final position 
    a = zeros(1,L); a(end) = 1;
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
[traj_smt,fval,exit_flag] = fmincon(...
    fun,...
    x_init,...
    A_ineq,b_ineq,...
    A_eq,b_eq,...
    [],[],[],opt);

if VERBOSE
    fprintf("[optimization_based_smoothing_1d] fmincon done. fval:[%.3f] \n",fval);
    print_fmincon_exit_flag(exit_flag);
end
