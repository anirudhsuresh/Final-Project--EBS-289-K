%% Variable Set-up
function[]=kalman_filter(path)

path=[0,0;0,0.200000000000000;0,0.400000000000000;0,0.600000000000000;0,0.800000000000000;0,1;0,1.20000000000000;0,1.40000000000000;0,1.60000000000000;0,1.80000000000000;0,2;0,2.20000000000000;0,2.40000000000000;0,2.60000000000000;0,2.80000000000000;0,3;0,3.20000000000000;0,3.40000000000000;0,3.60000000000000;0,3.80000000000000;0,4;0,4.20000000000000;0,4.40000000000000;0,4.60000000000000;0,4.80000000000000;0,5;0,5.20000000000000;0,5.40000000000000;0,5.60000000000000;0,5.80000000000000;0,6;0,6.20000000000000;0,6.40000000000000;0,6.60000000000000;0,6.80000000000000;0,7;0,7.20000000000000;0,7.40000000000000;0,7.60000000000000;0,7.80000000000000;0,8;0,8.20000000000000;0,8.40000000000000;0,8.60000000000000;0,8.80000000000000;0,9;0,9.20000000000000;0,9.40000000000000;0,9.60000000000000;0,9.80000000000000;0,10;0,10.2000000000000;0,10.4000000000000;0,10.6000000000000;0,10.8000000000000;0,11;0,11.2000000000000;0,11.4000000000000;0,11.6000000000000;0,11.8000000000000;0,12;0,12.2000000000000;0,12.4000000000000;0,12.6000000000000;0,12.8000000000000;0,13;0,13.2000000000000;0,13.4000000000000;0,13.6000000000000;0,13.8000000000000;0,14;0,14.2000000000000;0,14.4000000000000;0,14.6000000000000;0,14.8000000000000;0,15;0,15.2000000000000;0,15.4000000000000;0,15.6000000000000;0,15.8000000000000;0,16;0,16.2000000000000;0,16.4000000000000;0,16.6000000000000;0,16.8000000000000;0,17;0,17.2000000000000;0,17.4000000000000;0,17.6000000000000;0,17.8000000000000;0,18;0,18.2000000000000;0,18.4000000000000;0,18.6000000000000;0,18.8000000000000;0,19;0,19.2000000000000;0,19.4000000000000;0,19.6000000000000;0,19.8000000000000;0,20;0,20;0.200000000000000,20;0.400000000000000,20;0.600000000000000,20;0.800000000000000,20;1,20;1.20000000000000,20;1.40000000000000,20;1.60000000000000,20;1.80000000000000,20;2,20;2.20000000000000,20;2.40000000000000,20;2.60000000000000,20;2.80000000000000,20;3,20;3.20000000000000,20;3.40000000000000,20;3.60000000000000,20;3.80000000000000,20;4,20;4.20000000000000,20;4.40000000000000,20;4.60000000000000,20;4.80000000000000,20;5,20;5.20000000000000,20;5.40000000000000,20;5.60000000000000,20;5.80000000000000,20;6,20;6.20000000000000,20;6.40000000000000,20;6.60000000000000,20;6.80000000000000,20;7,20;7.20000000000000,20;7.40000000000000,20;7.60000000000000,20;7.80000000000000,20;8,20;8.20000000000000,20;8.40000000000000,20;8.60000000000000,20;8.80000000000000,20;9,20;9.20000000000000,20;9.40000000000000,20;9.60000000000000,20;9.80000000000000,20;10,20;10.2000000000000,20;10.4000000000000,20;10.6000000000000,20;10.8000000000000,20;11,20;11.2000000000000,20;11.4000000000000,20;11.6000000000000,20;11.8000000000000,20;12,20;12.2000000000000,20;12.4000000000000,20;12.6000000000000,20;12.8000000000000,20;13,20;13.2000000000000,20;13.4000000000000,20;13.6000000000000,20;13.8000000000000,20;14,20;14.2000000000000,20;14.4000000000000,20;14.6000000000000,20;14.8000000000000,20;15,20;15.2000000000000,20;15.4000000000000,20;15.6000000000000,20;15.8000000000000,20;16,20;16.2000000000000,20;16.4000000000000,20;16.6000000000000,20;16.8000000000000,20;17,20;17.2000000000000,20;17.4000000000000,20;17.6000000000000,20;17.8000000000000,20;18,20;18.2000000000000,20;18.4000000000000,20;18.6000000000000,20;18.8000000000000,20;19,20;19.2000000000000,20;19.4000000000000,20;19.6000000000000,20;19.8000000000000,20;20,20;20.2000000000000,20;20.4000000000000,20;20.6000000000000,20;20.8000000000000,20;21,20;21.2000000000000,20;21.4000000000000,20;21.6000000000000,20;21.8000000000000,20;22,20;22.2000000000000,20;22.4000000000000,20;22.6000000000000,20;22.8000000000000,20;23,20;23.2000000000000,20;23.4000000000000,20;23.6000000000000,20;23.8000000000000,20;24,20;24.2000000000000,20;24.4000000000000,20;24.6000000000000,20;24.8000000000000,20;25,20;25.2000000000000,20;25.4000000000000,20;25.6000000000000,20;25.8000000000000,20;26,20;26.2000000000000,20;26.4000000000000,20;26.6000000000000,20;26.8000000000000,20;27,20;27.2000000000000,20;27.4000000000000,20;27.6000000000000,20;27.8000000000000,20;28,20;28.2000000000000,20;28.4000000000000,20;28.6000000000000,20;28.8000000000000,20;29,20]
global bitmap points rangeMax occuGrid probGrid dT DT DTgps



%input and state constraints
Vmax = 4; %velocity max
turning_angle_max=55*(pi/180);
L = 3;                                                      % L is the Wheelbase of the robot

% % 
% % global pathroute

%simulation parameters
dT = 0.01; %Euler integration timestep
DT = 0.1; %Controller sampling timestep
DTgps = 1; %sampling time of GPS
T = 230; %simulation time
steps = T/DT; %number of steps in simulation
gpssamplerate = DTgps/DT; %ratio of gps sampling to controler interval


q_true = zeros(5, steps); % state vectors over N time steps 
q = zeros(5, steps); % state vectors over N time steps for kinematic model
u = zeros(2, steps); % input vectors over N time steps
q_pred = zeros(3, steps); %odometery values over N time steps

% % Pre-computed variance values
varianceOdo = [0.0025 1.1420e-04]; %varaince in noise from odometery (distance, theta)
varianceGPS = [8.8084e-04 9.5809e-04 4.0491e-04]; %variance in noise for x from gps (x, y, theta)%% Begin odometery and filtering 

% %initial pose of the robot

q_true(:,1) = [0,0,pi/2,0,0];
q(:,1) = [0,0,pi/2,0,0];
q_pred(:,1) = [0,0,pi/2];

% 
%Non-Ideal effects 
delta1 = 0*pi/180; delta2 = 0*pi/180; s = 0.0; %slip and skid
tau_g = 0.1; %time-lag for turning angle
tau_v = 0.2; %time-lag for velocity

%create constraint vectors
Qmax(1) = inf; Qmax(2)=inf; Qmax(3) = inf; %state constraint
Qmax(4) = turning_angle_max; Qmax(5) = Vmax;%state constraint
Qmin = -Qmax; %minimum constraints.
Umax = [turning_angle_max Vmax]'; %input constraint
Umin= -Umax;%input constraint

%Pursuit parameters
Ld = 2.0; %lookahead distance


steering_angle=1;
desired_velocity=2;
k=1; %counter for time

for t=0:DT:T-2*DT


%     dist = ((pathroute(1,:)-q_true(1,k)).^2 + (pathroute(2,:)-q_true(2,k)).^2).^(1/2); 
%     [Mc,Ic] = min(dist);
    
     if (q_true(2,k) < 1 && wrapTo2Pi(q_true(3,k)) > 1.2*pi) %makes sure the robot doesn't leave 1st quadrant
        u(2,k) = 0;
    else
        u(2,k) = 1; %desired velocity
    end

   
   q_pred(:,k)
   L
   Ld
   path
%     currentpath = pathroute(:,fix(max((min(u(2,k),Vmax)*t/spacing-1/spacing),1)):fix(min(min(u(2,k),Vmax)*t/spacing+1*Ld/spacing,length(pathroute)))).'; 
    [steer_angle, cross_track_error] = purePursuitController(q_pred(:,k),  L, Ld, path);
   
%     [gamma,error] = purePursuitController(q_pred(:,k),l,Ld,currentpath);

    u(steering_angle,k) = steer_angle;
%     u(desired_velocity,k) = 1; %desired velocity
   
    
    [q_true_next, odo_reading] = robot_odo(q_true(:,k), u(:,k), Umin, Umax, Qmin, Qmax, L, tau_g, tau_v);
   
    %computing Jcobians
    F_x = [1 0 -1*q_pred(1,k)*sin(q_pred(3,k)); 0, 1, q_pred(1,k)*cos(q_pred(3,k)); 0 0 1];
    F_v = [cos(q_pred(3,k)) 0; sin(q_pred(3,k)) 0; 0 1];
    P = zeros(3); %
    V = diag(varianceOdo); %odometery variance matrix, calculated previously
    W = diag(varianceGPS); %GPS variance matrix, calculated previously
    H_x = eye(3); %jacobian, here just idetiy matrix
    H_w = eye(3);%jacobian, here just idetiy matrix
    

    %Prediction step
    q_pred(:,k+1) = q_pred(:,k) + [odo_reading(1)*cos(q_pred(3,k)+odo_reading(2)); odo_reading(1)*sin(q_pred(3,k)+odo_reading(2)); odo_reading(2)];
    
    P = F_x*P*F_x' + F_v*V*F_v'; 
    q_true(:, k+1) = q_true_next;

    
    if mod(k-1,gpssamplerate) == 0 



        [x_measured, y__measured, theta_measured] = GPS_CompassNoisy(q_true(1,k+1),q_true(2,k+1),q_true(3,k+1));
        
        %Update step 
        V_innovovation = [x_measured; y__measured; theta_measured] - q_pred(:,k+1); 
        
        K_gain = (P*H_x')/(H_x*P*H_x'+H_w*W*H_w'); 
        
        q_pred(:,k+1) = q_pred(:,k+1)+ K_gain*V_innovovation; 
        P = P - K_gain*H_x*P; 
    end
       
    k = k+1;
    if (k == steps+1)
        k = k-1;
    end
    

end



%% Plot path
figure(2)
plot(q_true(1,:), q_true(2,:),'b');
plot(q_pred(1,:), q_pred(2,:),'r');
legend('Ground Truth','EKF Estimation')