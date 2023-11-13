function [A_vel,A_acc,A_jerk] = get_A_vel_acc_jerk(L,dt,varargin)
%
% Get matrices corresponding to velocity and acceleration space mapping
% When computing velocities, we assume that v@(t=L-1)=v@(t=L)
% When computing accerlations, we assume that v@(t=L+1)=0.0
% When computing jerks, a@(t=L-1)=a@(t=L)=a@(t=L+1)
%

% Velocity mapping
A_vel = zeros(L,L);
for i = 1:L
    if i <= (L-1)
        A_vel(i,i) = -1;
        A_vel(i,i+1) = 1;
    end
end
A_vel(end,:) = A_vel(end-1,:);
A_vel = 1/dt*A_vel;

% Acceleration mapping
A_acc = zeros(L,L);
for i = 1:L
    if i <= (L-2)
        A_acc(i,i) = 1;
        A_acc(i,i+1) = -2;
        A_acc(i,i+2) = 1;
    end
end
A_acc(end-1,:) = A_acc(end-2,:);
A_acc(end,:)   = A_acc(end-2,:);
A_acc = 1/dt/dt*A_acc;

% Jerk mapping
A_jerk = zeros(L,L);
for i = 1:L
    if i <= (L-3)
        A_jerk(i,i) = -1;
        A_jerk(i,i+1) = 3;
        A_jerk(i,i+2) = -3;
        A_jerk(i,i+3) = 1;
    end
end
A_jerk(end-2,:) = A_jerk(end-3,:);
A_jerk(end-1,:) = A_jerk(end-3,:);
A_jerk(end,:)   = A_jerk(end-3,:);
A_jerk = 1/dt/dt/dt*A_jerk;
