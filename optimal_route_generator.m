function []=optimal_route_generator()

clc
clf 
clear all
close all
%% Initial parameters :
% Vehicle parameters 
L = 3;                                                      % L is the Wheelbase of the robot
W_r=2;                                                      % W_r is the width of the robot 
gamma_max = deg2rad(55);                                    % maximum steering angle
R_min = (L/tan(gamma_max));                                 % Minimum turning radius

% Orchard parameters 
RL=20;                                                      % Length of each row 
W=3;                                                        % Width of each row 
K_rows=5;                                                   % Orchard rows 
N=K_rows+1;                                                 % Rows plus 1 so that we can go tru all rows

%% Node co-ordinates computation of the field 
% 
% Intially the robot is at 0,0, and facing north 90 degress 
% We have that mid point of West row is (20,20)
% We divide the orchard into top, middle and bottom nodes 
% Totally there are 20 nodes including start and end node 
% Here we compute the top, middle and bottom row's first node co-ordinates
% 
% X co-ordinates of first nodes of each row 
% X2 is W to the left from the given mid-point(20,20)

X(1,2)=17;                                                  % bottom row first node 
X(1,N+2)=X(1,2);                                            % middle row first node 
X(1,2*N+2)=X(1,2);                                          % Top row first node

% Y cordinates of first nodes of each node 
% Y2 is on the same axis as the given mid-point 

Y(1,2)=RL;                                                  % bottom row first node            
Y(1,N+2)=RL/2+RL;                                           % middle row first node
Y(1,2*N+2)=2*RL;                                            % Top row first node

% Calculation of all the other node X co-ordinates 
for i=2:N                                                   % since each co-ordinate is W away 
    X(1,i+1)=X(1,i)+W;                                      % southern/bottom row x co-ordinates
    X(1,N+i+1)=X(1,N+i)+W;                                  % middle row x co rodinates 
    X(1,2*N+i+1)=X(1,N+i)+W;                                % northern/top row x co-ordinates
end

% Calculation of all the other node Y co-ordinates 
for i=2:N                                                   % since each co-ordinate is W away
    Y(1,i+1)=RL;                                            % south y co-ordinates
    Y(1,N+i+1)=RL+RL/2;                                     % northern y co-ordinates
     Y(1,2*N+i+1)=2*RL;
end
X(1,1)=0;                                                   % start node x' co-ordinate
X(1,3*N+2)=1;                                               % end node   x' co-ordinate

Y(1,1)=0;                                                   % start node y' co-ordinate
Y(1,3*N+2)=0;                                               % end node   y' co-ordinate

%% Visual check of orchard / 
grid on
clf 
figure(1)
% Node plots
X_plot=X(1:19);
Y_plot=Y(1:19);
plot(X_plot,Y_plot,'o')                  
% title('Orchard Nodes to be traversed')
% xlabel('X axis')
% ylabel('Y axis')
% Number labels
x_1=0.5;
y_1=-0.05;
for i=1:20
    if i==1
        y_1=1;
    else 
        y_1=-0.05;
    end
    X_1(1,i)=X(1,i)+x_1;
    Y_1(1,i)=Y(1,i)+y_1;
end
for i=1:length(X)
    if i==20
        break
    end
    text(X_1(i),Y_1(i),num2str(i))
    hold on
end
% Node plots
figure(2)
hold on
for i=1:length(X)
    if i==20
        break
    end
    text(X_1(i),Y_1(i),num2str(i))
    hold on
end
%% Computation of the cost matrix DMAT
% Four parts :
% 1.Non turning Costs
% 2.Turning costs (pi/omega turns)
% 3.Start and end node to other nodes costs
% 4.Cost from same node to same node

DMAT=zeros(3*N+2,3*N+2);
% Soft constraints
huge=10^20;
small=-100;
sp_huge=10^25;


% 1. Non turning costs / straight lines
% 1.1 From bottom to middle row movement
for i=2:N+1                                                 % bottom row
    for j=N+2:2*N+1                                         % middle row
        if (j-i)==N                                         % both nodes are in same row
            DMAT(i,j)=small;
            DMAT(j,i)=small;
        else                                                % i and j are not of the same row:Assign cost to huge
            DMAT(i,j)=huge;
            DMAT(j,i)=huge;
        end
    end
end



% 1.2 From middle to top row movement 
for i=N+2:2*N+1                                             % bottom row
    for j=2*N+2:3*N+1                                       % topmost row
        if (j-i)==N                                         % both nodes are in same row
            DMAT(i,j)=small;
            DMAT(j,i)=small;
        else                                                % i and j are not of the same row:Assign cost to huge
%             to prevent going to another row's top node
            DMAT(i,j)=huge;
            DMAT(j,i)=huge;
        end
    end
    
end
% 1.3 From bottom row to top row 
% To prevent it from going directly to top row 
for i= 2:N+1
    for j=2*N+2:3*N+1
        DMAT(i,j) = huge;% we dont want to go from bottom to top
        DMAT(j,i) = huge;
    end
end


% 2. Turning costs 
% 2.1 pi and omega manuvers 

for i=2:N                                                   % Bottom row 
    for j=i+1:N+1                                             % Node to the right
        d = abs(i-j);                                       % Distance between nodes
        f=d*R_min;                                          % Turning radius bettwen node i and j
        if f>=2*R_min                                       % This is a pi turn
            cost=((d*W_r+(pi-2)*R_min)); 
            DMAT(i,j)=cost;     % Cost of a pi turn
        else % omega turn 
            
            cost=((3*pi*R_min-2*R_min*acos(1-(2*R_min+d*W_r)^2/(8*R_min^2))));            % Cost of a omgega turn
            DMAT(i,j)=cost;
        end   
     
%      Symmetrical conditions for topmost/northern row
     DMAT(j,i)=DMAT(i,j); 
     DMAT(i+2*N, j+2*N) = DMAT(i,j); 
     DMAT(j+2*N, i+2*N)= DMAT(i,j);
%      to prevent movement from middle node to another row middle node 
%      DMAT(N+i,N+j)=huge; 
%      DMAT(N+j,N+i)=huge;
    end
end

% for i=2:N
%     for j=2*N:3*N+1
%         DMAT(i,j)=huge;
%         DMAT(j,i)=huge;
%     end
% end



% 3. Start and End node costs 
s_x=[X(1,1),Y(1,1)];                            % start node co-ordinates 
e_x=[X(1,3*N+2),Y(1,3*N+2)];                       % end node co-ordinates 

% 3.1 Costs from the start node and end node to top and bottom nodes - we use manhattan distance
for i=2:3*N+1
    if (i>1 && i<N+1)
        DMAT(1,i) = small;
        DMAT(3*N+2,i) = small;
    else
        DMAT(1,i) =  abs(X(1)-X(i)) + abs(Y(1)-Y(i)); %manhattan distance
        DMAT(3*N+2,i) = abs(X(2*N+2)-X(i)) + abs(Y(2*N+2)-Y(i)); %manhattan distance
        DMAT(1,i) =huge; %manhattan distance
        DMAT(3*N+2,i) = huge; %manhattan distance
    end
    
    DMAT(i,1) = DMAT(1,i); % cost matrix symmetry
    DMAT(i,3*N+2) = DMAT(3*N+2,i); % cost matrix symmetry
end

% 3.2
% To prevent the robot to directly move into the middle rows 
% Or to prevent the robot to directly exit from the middle rows 
% We assign those costs to 'huge'
for i=8:13
    DMAT(1,i)=huge;
    DMAT(3*N+2,i)=huge;
    DMAT(1,i)=DMAT(i,1);
    DMAT(3*N+2,i)=DMAT(i,3*N+2);
    
end 

% % 3.3
% To prevent the robot from going direclty from start to end node 
% % Cost between start and end node is also assigned as 'huge'
DMAT(1,3*N+2) = huge;
DMAT(3*N+2, 1) = huge; 

% 4 Cost from a one 'node' to the same 'node'
for i=1:3*N+2
    for i=1:3*N+2
        DMAT(i,i)=sp_huge;
    end 
end 

%% Optimal node sequence 
% Using TSP-GA algorithm 

XY = [X' Y'];
t = cputime;  
resultStruct = tspof_ga('xy', XY , 'DMAT', DMAT, 'SHOWRESULT',false, 'SHOWWAITBAR', false, 'SHOWPROG', false); 
% E = cputime-t; %print time required to compute it.   
route = [1 resultStruct.optRoute 3*N+2]; % extract node sequence   
display(route)


%% Compute the path for Pure pursuit to follow 

path_generator(route,XY,R_min);
global Q_TRUE;



