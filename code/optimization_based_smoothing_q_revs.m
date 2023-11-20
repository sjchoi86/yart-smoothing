function [q_revs_smt,exit_flags] = optimization_based_smoothing_q_revs(...
    secs,q_revs,chain_robot,varargin)
%
% Optimization-based smoothing of revolute joint trajectories of a robot
%

% Parse options
iP = inputParser;
addParameter(iP,'vel_limit',[]);  % velocity limit (upper bound & lower bound)
addParameter(iP,'acc_limit',[]);  % acceleration limit (upper bound & lower bound)
addParameter(iP,'jerk_limit',[]); % jerk limit (upper bound & lower bound)
addParameter(iP,'vel_init',[]);    % initial velocity
addParameter(iP,'vel_final',[]);   % final velocity
addParameter(iP,'acc_init',[]);    % initial acceleration
addParameter(iP,'acc_final',[]);   % final acceleration
addParameter(iP,'fade_dur',[]);    % fade in and out duration in seconds
addParameter(iP,'fade_jerk_limit',[]); % fade in and out jerk limit
addParameter(iP,'max_iter',10000); 
addParameter(iP,'optm_tol',1e-6); 
addParameter(iP,'step_tol',1e-6); 
addParameter(iP,'CHECK_SC',false); 
addParameter(iP,'VERBOSE',true); 
parse(iP,varargin{:});
vel_limit  = iP.Results.vel_limit;
acc_limit  = iP.Results.acc_limit;
jerk_limit = iP.Results.jerk_limit;
vel_init   = iP.Results.vel_init;
vel_final  = iP.Results.vel_final;
acc_init   = iP.Results.acc_init;
acc_final  = iP.Results.acc_final;
fade_dur   = iP.Results.fade_dur;
fade_jerk_limit = iP.Results.fade_jerk_limit;
max_iter   = iP.Results.max_iter;
optm_tol   = iP.Results.optm_tol;
step_tol   = iP.Results.step_tol;
CHECK_SC   = iP.Results.CHECK_SC;
VERBOSE    = iP.Results.VERBOSE;

n_joint = size(q_revs,2); % number of joints
q_revs_smt = q_revs;
if VERBOSE
    fprintf("[optimization_based_smoothing_q_revs] Smooth [%d] joint trajectories.\n",...
        n_joint);
end
exit_flags = zeros(1,n_joint);
for i_idx = 1:n_joint % for each joint

    % Get joint position limits
    joint_name = chain_robot.rev_joint_names{i_idx};
    joint_idx = idx_cell(chain_robot.joint_names,joint_name);
    joint_limit = chain_robot.joint(joint_idx).limit;

    % Current joint trajectory to smooth
    traj = q_revs(:,i_idx);

    % Apply joint position limits
    pos_lower  = joint_limit(1); % -3;
    pos_upper  = joint_limit(2); % +3;

    % Enforce the smoothed trajectories to have the same initial and final positions
    pos_init   = traj(1);
    pos_final  = traj(end);

    % Run smoothing
    if VERBOSE
        fprintf(" [%d/%d] smooting joint_name:[%s] ...",i_idx,n_joint,joint_name);
    end
    [traj_smt,exit_flag] = optimization_based_smoothing_1d(...
        secs,traj,...
        'pos_lower',pos_lower,'pos_upper',pos_upper,...
        'vel_limit',vel_limit,...
        'acc_limit',acc_limit,...
        'jerk_limit',jerk_limit,...
        'pos_init',pos_init,...
        'pos_final',pos_final,...
        'vel_init',vel_init,'vel_final',vel_final,...
        'acc_init',acc_init,'acc_final',acc_final,...
        'fade_dur',fade_dur,'fade_jerk_limit',fade_jerk_limit,...
        'max_iter',max_iter,'optm_tol',optm_tol,'step_tol',step_tol,...
        'VERBOSE',false);
    if VERBOSE
        fprintf(" done.\n");
        if exit_flag < 0
            print_fmincon_exit_flag(exit_flag);
        end
    end

    % Append
    q_revs_smt(:,i_idx) = traj_smt;
    exit_flags(i_idx) = exit_flag;
end

% Check selc-collision
if CHECK_SC
    if VERBOSE
        fprintf(" Start self-collision chekcing...\n");
    end
    L = size(q_revs_smt,1);
    chains = cell(1,L);
    SC_happened = false;
    for tick = 1:L
        q_rev = q_revs_smt(tick,:);
        T_root = eye(4,4);
        chain_robot = update_chain_q_root_T(chain_robot,q_rev,T_root);
        chains{tick} = chain_robot;
        SC = check_sc(chain_robot);
        if SC
            SC_happened = true;
            if VERBOSE
                fprintf(2," [%d/%d] Self-collision occurs! \n",tick,L);
            end
        end
    end
    if VERBOSE
        if SC_happened
            fprintf(2," Self-collision occurred!\n");
        else
            fprintf(" Self-collision Free.\n");
        end
    end
end