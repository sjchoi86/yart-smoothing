function [secs,q_revs,T_roots,chain_robot] = load_motion(...
    motion_idx,varargin)
%
% Load motion
%

% Parse options
iP = inputParser;
addParameter(iP,'VERBOSE',true);
parse(iP,varargin{:});
VERBOSE = iP.Results.VERBOSE;

% Parse existing motions
motion_paths = dir_compact('../data/mhformer_*','VERBOSE',0);
n_motion = length(motion_paths);
mat_path = [motion_paths(motion_idx).folder,'/',motion_paths(motion_idx).name];

% Load motion
l = load(mat_path);
chain_robot = l.chain_robot; secs = l.secs;
q_revs = l.q_revs_robot_sch; T_roots = l.T_roots_robot_sch;

if VERBOSE
    fprintf("We have [%d] motions.\n",n_motion);
    fprintf("We will be using [%d/%d]-th motion from [%s]. \n",...
        motion_idx,n_motion,mat_path);
end
