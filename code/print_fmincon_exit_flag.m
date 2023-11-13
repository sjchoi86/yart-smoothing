function print_fmincon_exit_flag(exit_flag)
%
% Print the meaning of the exit flag of fmincon
%
switch exit_flag
    case 3
        fprintf(" Change in objective function too small.\n");
    case 2
        fprintf(" Change in X too small.\n");
    case 1
        fprintf(" First order optimality conditions satisfied.\n");
    case 0
        fprintf(" Too many function evaluations or iterations.\n");
    case -1
        fprintf(2," Stopped by output/plot function.\n");
    case -2
        fprintf(2," No feasible point found.\n");
end
