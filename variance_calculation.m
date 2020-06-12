clc
clear 
true_position=[0;0;0];
%% gps variance calculation
for i=1:10000
[x_noisy, y_noisy, theta_noisy] = GPS_CompassNoisy(true_position(1),true_position(2),true_position(3));
noise_in_x(:,i)=true_position(1)-x_noisy;
noise_in_y(:,i)=true_position(2)-y_noisy;
noise_in_t(:,i)=true_position(3)-theta_noisy;
end
%variance calculation
C_x=cov(noise_in_x);
C_y=cov(noise_in_y);
C_t=cov(noise_in_t);
Covariance_gps=[C_x C_y C_t];

%% robot odo variance calculation :
global dT;
global DT;
dT = 0.01; DT = 0.01; 

tau_g = 0.1; %time-lag for turning angle
tau_v = 0.2; %time-lag for velocity
gamma_max = deg2rad(55); 
Vmax=4;
L = 3; 

% start_pos=start_pos;
Qmax(1) = inf; Qmax(2)=inf; Qmax(3) = inf; %state constraint
Qmax(4) = gamma_max; Qmax(5) = Vmax;%state constraint
Qmin = -Qmax; %minimum constraints.
Umax = [gamma_max Vmax]'; %input constraint
Umin= -Umax;%input constraint
q_true=[0;0;0;0;0];
u=[0 1];
T=100; % becasue we want 10000 iterations

check=1;
for t = 0:DT:T-DT
[q_true_next, odo_reading] = robot_odo(q_true, u, Umin, Umax, Qmin, Qmax, L, tau_g, tau_v);
distance_travelled = sqrt( (q_true_next(1)- q_true(1))^2 + (q_true_next(2)- q_true(2))^2);
angle_change = q_true_next(3) - q_true(3);

noisy_o_1(:,check)=distance_travelled-odo_reading(1);
noisy_o_2(:,check)=angle_change-odo_reading(2); 
check=check+1;
end
%variance calculation
O_x=cov(noisy_o_1);
O_t=cov(noisy_o_2);
Covariance_odo=[O_x O_t];
