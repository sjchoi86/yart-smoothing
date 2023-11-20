function [secs_tween,traj_tween,exit_flag] = automated_tweening_1d(...
    secs_fr,traj_fr,secs_to,traj_to,varargin)
%
% Automated tweening using 1D
%

% Parse options
iP = inputParser;
addParameter(iP,'pos_lower',[]); % position upper bound
addParameter(iP,'pos_upper',[]);  % position lower bound

addParameter(iP,'vel_limit',[]);  % velocity limit (upper bound & lower bound)
addParameter(iP,'acc_limit',[]);  % acceleration limit (upper bound & lower bound)
addParameter(iP,'jerk_limit',[]); % jerk limit (upper bound & lower bound)

addParameter(iP,'vel_limit_rate',1.0);
addParameter(iP,'acc_limit_rate',0.5);
addParameter(iP,'jerk_limit_rate',0.5);
addParameter(iP,'dur_tweenings',[2,5,10]);
addParameter(iP,'VERBOSE',true);
parse(iP,varargin{:});
pos_lower       = iP.Results.pos_lower;
pos_upper       = iP.Results.pos_upper;

vel_limit       = iP.Results.vel_limit;
acc_limit       = iP.Results.acc_limit;
jerk_limit      = iP.Results.jerk_limit;

vel_limit_rate  = iP.Results.vel_limit_rate;
acc_limit_rate  = iP.Results.acc_limit_rate;
jerk_limit_rate = iP.Results.jerk_limit_rate;
dur_tweenings   = iP.Results.dur_tweenings;
VERBOSE         = iP.Results.VERBOSE;

% Equality and inequality contraints
[vel_fr,acc_fr,jerk_fr] = get_vel_acc_jerk(secs_fr,traj_fr);
[vel_to,acc_to,jerk_to] = get_vel_acc_jerk(secs_to,traj_to);
if isempty(vel_limit)
    vel_limit  = max(abs([vel_fr;vel_to]))*vel_limit_rate;
end
if isempty(acc_limit)
    acc_limit  = max(abs([acc_fr;acc_to]))*acc_limit_rate;
end
if isempty(jerk_limit)
    jerk_limit = max(abs([jerk_fr;jerk_to]))*jerk_limit_rate;
end
pos_init   = traj_fr(end);
pos_final  = traj_to(1);
vel_init   = vel_fr(end);
vel_final  = vel_to(1);
acc_init   = acc_fr(end);
acc_final  = acc_to(1);

% Run optimization
for dur_tweening = dur_tweenings
    if VERBOSE
        fprintf("[automated_tweening_1d] dur_tweening:[%.1f]sec.\n",dur_tweening);
    end
    [secs_tween,traj_tween,exit_flag] = optimization_based_tweening_1d(...
        secs_fr,traj_fr,secs_to,traj_to,dur_tweening,...
        'pos_lower',pos_lower,'pos_upper',pos_upper,'vel_limit',vel_limit,...
        'acc_limit',acc_limit,'jerk_limit',jerk_limit,...
        'pos_init',pos_init,'pos_final',pos_final,...
        'vel_init',vel_init,'vel_final',vel_final,...
        'acc_init',acc_init,'acc_final',acc_final,...
        'VERBOSE',VERBOSE);
    if exit_flag >= 0
        break
    end
end

