function [final_pose,final_pred,P_end]=closed_loop_control(path,Ld,T,q_true,q_pred,P_ini)


% varianceOdo = [0.00250565682347454,0.000112850577164281]; %varaince in noise from odometery (distance, theta)
% varianceGPS = [0.000890899223534857,0.000911853781427416,0.000396694884195806]; %variance in noise for x from gps (x, y, theta)
% varianceGPS=[0.09036e-3 0.08778e-3 0.4003e-3];
% varianceGPS=[0.000908108848884272,0.000897047310024325,0.000387759389349412];
%vehicle parameters :
L = 3; 
varianceGPS=[0.000885970690691681,0.000886540428368619,0.000399979236563813];
varianceOdo=[0.00243157008095900,0.000110309828567493];
%create constraint vectors
gamma_max = deg2rad(55); 
Vmax=4;
v=1;

global dT;
global DT;
global Q_TRUE;
dT = 0.01; DT = 0.01; 

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

V = diag(varianceOdo); %odometery variance matrix, calculated previously
W = diag(varianceGPS); %GPS variance matrix, calculated previously %noise covariance
H_x = eye(3);
H_w = eye(3);

% P = zeros(3); %System uncertainity
P=P_ini;


h = waitbar(0, 'Calculating Path Traversed');
for t = 0:DT:T
    %% Check for if goal position is reached 
    e_x=current_position(1)-end_position(1); % differnce bettwen x co ordinates of goal and current point 
    e_y=current_position(2)-end_position(2);% differnce bettwen y co ordinates of goal and current point 
    if abs(e_x+e_y)<epilison % if the current point is near goal point stop / break out of loop
        disp('position reached- Terminating closed loop');
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
    
    F_x = [1 0 -1*q_pred(1)*sin(q_pred(3)); 0, 1, q_pred(1)*cos(q_pred(3)); 0 0 1];
    F_v = [cos(q_pred(3)) 0; sin(q_pred(3)) 0; 0 1];
%     V = diag(varianceOdo); %odometery variance matrix, calculated previously
%     W = diag(varianceGPS); %GPS variance matrix, calculated previously %noise covariance
%     H_x = eye(3); 
%     H_w = eye(3);
    %% EKF Prediction Equations
    q_pred_next = q_pred + [odo_reading(1)*cos(q_pred(3)+odo_reading(2)); odo_reading(1)*sin(q_pred(3)+odo_reading(2)); odo_reading(2)];
    
    P = F_x*P*F_x' + F_v*V*F_v';
    q_true = q_true_next;
    
    
    %% GPS avaliable 
    if mod(count,100) == 0 %if a gps singal is avaliable
        
        
        % Get the gps measurements for the next time step
        % the real sesnor measurements
        [x_measured, y_measured, theta_measured] = GPS_CompassNoisy(q_true_next(1),q_true_next(2),q_true_next(3));
        
        %% Sensing and Kalman gain equations
        
        % compute the innovation
        V_innovovation = [x_measured; y_measured; theta_measured] - q_pred_next;%innovation
        
        S=(H_x*P*H_x'+H_w*W*H_w');%innovation covarinace
        K_gain = (P*H_x')/S; %Kalman Gain
        
        %% Update Equation
        q_pred_next = q_pred_next+ K_gain*V_innovovation;
        P = P - K_gain*H_x*P; %update the uncertainity
    end
    %% Other steps
    robot_tracker_true(count,1:2) = q_true(1:2);
    robot_tracker_pred(count,1:2) = q_pred(1:2);
    P_end=P;
    q_pred=q_pred_next;
    
    waitbar(t/T);
    Q_TRUE(:,count)=q_true;
%     q_true = q_true_next;
%     q_pred=q_pred_next;
    current_position=[q_true(1);q_true(2)];
    count = count+1;
    
end
final_pose=[q_true(1);q_true(2);q_true(3)];
final_pred=[q_pred(1);q_pred(2);q_pred(3)];
close(h);

figure(1); hold on; axis equal;
plot(robot_tracker_true(1:end,1)', robot_tracker_true(1:end,2)', 'k')
plot(robot_tracker_pred(1:end,1)', robot_tracker_pred(1:end,2)', 'b')
title('True path VS Predicted path')
xlabel('X axis')
ylabel('Y axis')
legend('Nodes','Ground Truth','EKF Estimation')




