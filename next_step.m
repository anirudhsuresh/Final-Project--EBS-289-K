generateNursery(); %creates a random nursery, saving tree locations to bitmap global variable
bitmapinput = bitmap;% saves bitmap to a input variable for laser scanner
%% Variable Set-up

%simulation parameters
dT = 0.01; %Euler integration timestep
DT = 0.1; %Controller sampling timestep
DTgps = 1; %sampling time of GPS
T = 230; %simulation time
steps = T/DT; %number of steps in simulation
gpssamplerate = DTgps/DT; %ratio of gps sampling to controler interval
% Pre-computed variance values
varianceOdo = [0.0025 1.1420e-04]; %varaince in noise from odometery (distance, theta)
varianceGPS = [8.8084e-04 9.5809e-04 4.0491e-04]; %variance in noise for x from gps (x, y, theta)

%Space variables
Xmax = 50; Ymax = 50; %physical dimensions of space (m)
Xmax_nurs = 42; % xmax in the generateNursery script 
Ymax_nurs = Xmax_nurs; % xmax in the generateNursery script
R = 350; C = 350; %rows and columns; discretization of physical space in a grid
q_true = zeros(5, steps); % state vectors over N time steps 
q = zeros(5, steps); % state vectors over N time steps for kinematic model
u = zeros(2, steps); % input vectors over N time steps
e = zeros(1, steps); %cross track error
odo = zeros(3, steps); %odometery values over N time steps
occuGrid = ones(R, C); %initialize as odds of one for each pixel
probGrid = 0.0* ones(R, C); %initialize as empty

%Variance computation arrays
x_diff_gps = zeros(1, steps); %holds difference b/w gps data and q true for x
y_diff_gps = zeros(1, steps); %holds difference b/w gps data and q true for y
theta_diff_gps = zeros(1, steps); %holds difference b/w gps data and q true for angle
d_diff = zeros(1, steps); %holds difference b/w odometery data and q true for distance
theta_diff = zeros(1, steps); %holds difference b/w odometery data and q true for theta

%input and state constraints
Vmax = 4; %velocity max
gmax = 55*(pi/180); %turning angle max

%Feild parameters
rowWidth = 3;%spacing between tree rows
Krows = 5; % number of rows
N = Krows+1; % k plus 1 (so that robot encircles orchard)
len = N*rowWidth; %transverse to rows
RL = 20; %lateral length
nTrees = 11; %number of trees in the row
Treespacing = RL/(nTrees-1); %spacing between trees down the row
nStart = [17;20]; %one half row width to the left of the SW tree
trees_percieved = zeros((Krows*nTrees),3); %arry that holds the values of trees percieved for error calculation

%Path variables
nd = zeros(2,3*N+2); %nd for path
x = nd(1,:); %x coordinates of node points
y = nd(2,:); %y coordinates of node points
spacing = 0.1; %m, spacing between points on the path

%initial pose of the robot
q_true(:,1) = [25,0,pi/2,0,0];
q(:,1) = [0,0,pi/2,0,0];
odo(:,1) = [0,0,pi/2];

%Non-Ideal effects 
delta1 = 0*pi/180; delta2 = 0*pi/180; s = 0.0; %slip and skid
tau_g = 0.1; %time-lag for turning angle
tau_v = 0.2; %time-lag for velocity

%create constraint vectors
Qmax(1) = inf; Qmax(2)=inf; Qmax(3) = inf; %state constraint
Qmax(4) = gmax; Qmax(5) = Vmax;%state constraint
Qmin = -Qmax; %minimum constraints.
Umax = [gmax Vmax]'; %input constraint
Umin= -Umax;%input constraint

%Pursuit parameters
Ld = 2.0; %lookahead distance


%Kalman Filter parameters
Ks = 1.1; %steering angle gain
tauFilter = 0.5; %time constant of low pass filter

wTr = eye(3,3); %3x3 Matrix, transforamtion to the origin

%% Begin odometery and filtering
V = diag(varianceOdo); %odometery variance matrix, calculated previously
W = diag(varianceGPS); %GPS variance matrix, calculated previously
Hx = eye(3); %jacobian, here just idetiy matrix
Hw = eye(3);%jacobian, here just idetiy matrix
P = zeros(3); % a priori assumption of covariance matrix (zero b/c we know initial state perfectly)
k=1

Fx=[1,  0,  -1*(odo(1,k))*sin(odo(3,k))
    0,  1,  odo(1,k)*cos(odo(2,k))
    0,  0,  1 ];
Fv = [ cos(odo(3,k)),   0
    sin(odo(3,k)),   0
    0,               1];

% do pure purepursit controller
% somehow do purePC

[kappa,error]=purePursuitController(odo(:,k),l,Ld,currentpath);
u(1,k)=kappa;
if mod(k-1,gpssamplerate) == 0 %every get GPS data and perform EKF calculations
    
    [x_n, y_n, theta_n] = GPS_CompassNoisy(q_true(1,k),q_true(2,k),q_true(3,k)); %retrieves GPS data
    
    %calls kinematic model, which returns true state and noisy odometry
    [q_true_next, odo_next] = robot_odo(q_true(:,k), u(:,k), Umin, Umax, Qmin, Qmax, l, tau_g, tau_v);
    
    % Performs calculation to find covaraiance matricies
    x_diff_q = q_true_next(1)-q_true(1,k); %difference in x position since last step
    y_diff_q = q_true_next(2)-q_true(2,k); %difference in y position since last step
    distance_diff_q = sqrt((q_true_next(1)-q_true(1,k))^2+(q_true_next(2)-q_true(2,k))^2); %difference in euclidian distance since last step
    angle_diff_q = q_true_next(3) - q_true(3,k); %difference in pointing angle since last step
    
    % Following arrays store differnce between true and odo/gps data
    d_diff(k) = distance_diff_q - odo_next(1); %difference b/w odometery and q true for distance
    theta_diff(k) = angle_diff_q - odo_next(2); %difference b/w odometery and q true for angle
    x_diff_gps(k) = q_true(1,k) - x_n; % difference b/w gps and q true for x value
    y_diff_gps(k) = q_true(2,k) - y_n; % difference b/w gps and q true for y value
    theta_diff_gps(k) = q_true(3,k) - theta_n; %differnce b/w gps and q true for theta
    % EKF prediction step
    
    %update state estimate
    odo(:,k+1) = odo(:,k) + [odo_next(1)*cos(odo(3,k)+odo_next(2)); odo_next(1)*sin(odo(3,k)+odo_next(2)); odo_next(2)];
    
    P = Fx*P*Fx' + Fv*V*Fv'; %update estimate of state uncertainty
    q_true(:, k+1) = q_true_next; %update true position
    
    % Compute Kalman Gain
    S = Hx*P*Hx'+Hw*W*Hw'; %innovation covariance
    K = P*Hx'/S; %compute kalman gain
    
    %Compute innovation
    [x_n_next, y_n_next, theta_n_next] = GPS_CompassNoisy(q_true(1,k+1),q_true(2,k+1),q_true(3,k+1)); %calls GPS data for next step
    v_innov = [x_n_next; y_n_next; theta_n_next] - odo(:,k+1); %calculates innovation
    
    %Update Step
    odo(:,k+1) = odo(:,k+1)+ K*v_innov; %update state estimation
    P = P - K*Hx*P; %update covariance matrix
    
else %no GPS data for this step, integrate odometery and update P
    
    if mod(k-2,10) == 0 %step after GPS data is taken, saves point for tree localization
        qcurrent = [odo(1,k), odo(2,k), odo(3,k)]; %current true position
        
        if (odo(1,k) > 0 && odo(2,k) > 0) %if x and y values are positive
            points = vertcat(points,qcurrent); %adds point to points array (to pass to tree localization script)
        end
        
    end
    
    [q_true_next, odo_next] = robot_odo(q_true(:,k), u(:,k), Umin, Umax, Qmin, Qmax, l, tau_g, tau_v); %calls kinematic model
    odo(:,k+1) = odo(:,k) + [odo_next(1)*cos(odo(3,k)+odo_next(2)); odo_next(1)*sin(odo(3,k)+odo_next(2)); odo_next(2)]; %updates state estimation
    P = Fx*P*Fx' + Fv*V*Fv'; %update estimate of state uncertainty
    q_true(:, k+1) = q_true_next; %updates tue pose
end


