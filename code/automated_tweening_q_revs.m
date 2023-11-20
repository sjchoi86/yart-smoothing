function [secs_tween,q_revs_tween,secs_total,q_revs_total,exit_flags] = ...
    automated_tweening_q_revs(...
    secs_fr,q_revs_fr,secs_to,q_revs_to,chain_robot,varargin)
%
% Automated tweening using revolute joint trajectories
%

% Parse options
iP = inputParser;
addParameter(iP,'dur_tweenings',[2,5,10]); 
addParameter(iP,'vel_limit',[]);  % velocity limit (upper bound & lower bound)
addParameter(iP,'acc_limit',[]);  % acceleration limit (upper bound & lower bound)
addParameter(iP,'jerk_limit',[]); % jerk limit (upper bound & lower bound)
addParameter(iP,'pos_limit_margin',0.0873);
addParameter(iP,'PLOT_EACH_JOINT_TWEEN',false);
addParameter(iP,'CHECK_SC',true);
addParameter(iP,'VERBOSE',true);
parse(iP,varargin{:});
dur_tweenings         = iP.Results.dur_tweenings;
vel_limit             = iP.Results.vel_limit;
acc_limit             = iP.Results.acc_limit;
jerk_limit            = iP.Results.jerk_limit;
pos_limit_margin      = iP.Results.pos_limit_margin;
PLOT_EACH_JOINT_TWEEN = iP.Results.PLOT_EACH_JOINT_TWEEN;
CHECK_SC              = iP.Results.CHECK_SC;
VERBOSE               = iP.Results.VERBOSE;

% Automated in-betweening
n_joint = size(q_revs_fr,2);
for dur_tweening = dur_tweenings % for different tweening durations

    exit_flags = zeros(1,n_joint);
    q_revs_tween = [];
    for i_idx = 1:n_joint % for each joint

        % Get joint position limits
        joint_name = chain_robot.rev_joint_names{i_idx};
        joint_idx = idx_cell(chain_robot.joint_names,joint_name);
        joint_limit = chain_robot.joint(joint_idx).limit;

        % Current joint trajectory to smooth
        traj_fr = q_revs_fr(:,i_idx);
        traj_to = q_revs_to(:,i_idx);

        % Apply joint position limits
        pos_lower    = joint_limit(1)-pos_limit_margin; % -3;
        pos_upper    = joint_limit(2)+pos_limit_margin; % +3;

        % Automated tweening
        [secs_tween,traj_tween,exit_flag] = automated_tweening_1d(...
            secs_fr,traj_fr,secs_to,traj_to,...
            'pos_lower',pos_lower,'pos_upper',pos_upper,...
            'vel_limit',vel_limit,'acc_limit',acc_limit,'jerk_limit',jerk_limit,...
            'vel_limit_rate',1.0,'acc_limit_rate',0.5,'jerk_limit_rate',0.25,...
            'dur_tweenings',dur_tweening,'VERBOSE',VERBOSE);

        % Append
        q_revs_tween = [q_revs_tween, traj_tween];
        exit_flags(i_idx) = exit_flag;

        % Plot (debug)
        if PLOT_EACH_JOINT_TWEEN
            % Check smoothing
            check_tweening_results(...
                secs_fr,traj_fr,secs_tween,traj_tween,secs_to,traj_to,...
                'pos_lower',pos_lower,'pos_upper',pos_upper,...
                'vel_limit',vel_limit,'acc_limit',acc_limit,'jerk_limit',jerk_limit);
            % pause; 
            ca;
        end

    end % for i_idx = 1:n_joint % for each joint

    if sum(exit_flags<0) == 0 % break point
        fprintf("dur_tweening:[%.1f]sec looks okay. break. \n",dur_tweening)
        break;
    end

end % for dur_tweening = dur_tweenings % for different tweening durations

% Check self-collision
if CHECK_SC
    if VERBOSE
        fprintf(" Start self-collision chekcing...\n");
    end
    L = size(q_revs_tween,1);
    chains = cell(1,L);
    SC_happened = false;
    for tick = 1:L
        q_rev = q_revs_tween(tick,:);
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

% Interpolate 'q_revs_fr_smt', 'q_revs_tween', 'q_revs_to_smt' 
secs_in = [secs_fr;max(secs_fr)+secs_tween;max(secs_fr)+max(secs_tween)+secs_to];
traj_in = [q_revs_fr; q_revs_tween; q_revs_to];
sec_max = max(secs_fr)+max(secs_tween)+max(secs_to);
HZ = round(1/(secs_fr(2)-secs_fr(1)));
secs_total = linspace(0,sec_max,round(sec_max*HZ))';
q_revs_total = gp_based_interpolation(secs_in,traj_in,secs_total,'hyp_mu',[1,0.1]);
