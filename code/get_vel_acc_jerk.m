function [vel,acc,jerk] = get_vel_acc_jerk(secs,traj)
%
% Get velocity, acceleration, and jerk
%

L = size(secs,1);
dt = secs(2)-secs(1);
[A_vel,A_acc,A_jerk] = get_A_vel_acc_jerk(L,dt);

vel = A_vel*traj;
acc = A_acc*traj;
jerk = A_jerk*traj;
