function cost = get_tween_smoothing_cost(...
    x_a,x_b,x_c,A_vel,A_acc,A_jerk,varargin)
%
% 1D trajectory x is partitioned into three parts, a, b, and c, with lengths of n_a, n_b, and n_c.
% This function outputs the maximum acceleration of the middle parts, i.e., part b. 
%


% Parse options
iP = inputParser;
addParameter(iP,'cost_type','a'); % 'v', 'a', 'j'
parse(iP,varargin{:});
cost_type = iP.Results.cost_type;

% Concat
x_concat = [x_a;x_b;x_c];
switch cost_type
    case 'v'
        vel = A_vel*x_concat;
        cost = norm(vel); % max(abs(vel));
    case 'a'
        acc = A_acc*x_concat;
        cost = norm(acc); % max(abs(acc));
    case 'j'
        jerk = A_jerk*x_concat;
        cost = norm(jerk); % max(abs(jerk));
    otherwise
        fprintf(2,"[get_tween_smoothing_cost] undefined cost_type:[%s]\n",cost_type);
        cost = 0;
end
