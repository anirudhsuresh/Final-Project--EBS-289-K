function [final_pose,final_pred]=closed_loop_control(path,Ld,T,q_true,q_pred)


varianceOdo = [0.0025 1.1420e-04]; %varaince in noise from odometery (distance, theta)
varianceGPS = [8.8084e-04 9.5809e-04 4.0491e-04]; %variance in noise for x from gps (x, y, theta)
%equate the start position for the current path 

gpssamplerate=100;


%vehicle parameters :
L = 3; 

%create constraint vectors
gamma_max = deg2rad(55); 
Vmax=4;
v=1;

global dT;
global DT;
dT = 0.001; DT = 0.01; 

tau_g = 0.1; %time-lag for turning angle
tau_v = 0.2; %time-lag for velocity

% start_pos=start_pos;
Qmax(1) = inf; Qmax(2)=inf; Qmax(3) = inf; %state constraint
Qmax(4) = gamma_max; Qmax(5) = Vmax;%state constraint
Qmin = -Qmax; %minimum constraints.
Umax = [gamma_max Vmax]'; %input constraint
Umin= -Umax;%input constraint



q_pred_next=zeros();
robot_tracker_true = zeros();% for plotting the trace of the q_true
robot_tracker_pred=zeros();
count = 1; % count for the trace

epilison=0.1; % the threshold for the robot to stop moving 

current_position=[q_true(1);q_true(2)];% current robot position
end_position=[path(end,1);path(end,2)]; % end positon that the robot needs to stop at






h = waitbar(0, 'Calculating path');
for t = 0:DT:T
    %% Check for if goal position is reached 
    e_x=current_position(1)-end_position(1); % differnce bettwen x co ordinates of goal and current point 
    e_y=current_position(2)-end_position(2);% differnce bettwen y co ordinates of goal and current point 
    if abs(e_x+e_y)<epilison % if the current point is near goal point stop / break out of loop
        disp('position reached');
%         f = msgbox('position reached - Terminating closed loop');
        break % break out of loop since goal point has been reached
    end 
    %% Pure Pursuit Controller 
    [gamma, ~] = purePursuitController(q_true, L, Ld, path);
    u = [gamma v];
    
    %% Robot odo
   [q_true_next, odo_reading] = robot_odo(q_true, u, Umin, Umax, Qmin, Qmax, L, tau_g, tau_v);   
    

    
    %% EKF 
    %% initial variables 
    % Caculations of required Jacobians and other input parameters for EKF    
    P = zeros(3); %System uncertainity
    F_x = [1 0 -1*q_pred(1)*sin(q_pred(3)); 0, 1, q_pred(1)*cos(q_pred(3)); 0 0 1];
    F_v = [cos(q_pred(3)) 0; sin(q_pred(3)) 0; 0 1];
    V = diag(varianceOdo); %odometery variance matrix, calculated previously
    W = diag(varianceGPS); %GPS variance matrix, calculated previously %noise covariance
    H_x = eye(3); 
    H_w = eye(3);
    %% EKF Prediction Equations
    q_pred_next = q_pred + [odo_reading(1)*cos(q_pred(3)+odo_reading(2)); odo_reading(1)*sin(q_pred(3)+odo_reading(2)); odo_reading(2)];
    
    P = F_x*P*F_x' + F_v*V*F_v';
    
    %% GPS avaliable 
    if mod(count,100) == 0 %if a gps singal is avaliable
        
        
        % Get the gps measurements for the next time step
        % the real sesnor measurements
        [x_measured, y__measured, theta_measured] = GPS_CompassNoisy(q_true_next(1),q_true_next(2),q_true_next(3));
        
        %% Sensing and Kalman gain equations
        
        % compute the innovation
        V_innovovation = [x_measured; y__measured; theta_measured] - q_pred_next;%innovation
        
        S=(H_x*P*H_x'+H_w*W*H_w');%innovation covarinace
        K_gain = (P*H_x')/S; %Kalman Gain
        
        %% Update Equation
        q_pred_next = q_pred_next+ K_gain*V_innovovation;
        P = P - K_gain*H_x*P; %update the uncertainity
    end
    %% Other steps
    robot_tracker_true(count,1:2) = q_true(1:2);
    robot_tracker_pred(count,1:2) = q_pred(1:2);
    
    
    waitbar(t/T);
    q_true = q_true_next;
    q_pred=q_pred_next;
    current_position=[q_true(1);q_true(2)];
    count = count+1;
    
end
final_pose=[q_true(1);q_true(2);q_true(3)];
final_pred=[q_pred(1);q_pred(2);q_pred(3)];
close(h);

figure(1); hold on; axis equal;
plot(robot_tracker_true(1:end,1)', robot_tracker_true(1:end,2)', 'k')
plot(robot_tracker_pred(1:end,1)', robot_tracker_pred(1:end,2)', 'b')



