%% Function: path_generator
% This function accepts the optimal route and generates waypoints in order for the purepursuit controller to follow 
% Input : optimal route , XY co ordinates and the minimum turning radius R_min
% Outputs: Generates the path between to nodes and passes it to the kalman fliter fuction in order to do odometery and flitering 


function []=path_generator(route,XY,R_min)
% route_taken=[ 1     3     9    15    18    12     6     4    10    16    19    13     7     5    11    17    14     8     2    20];

%% Initial Variables

pts=25; % number of points between a given start and end point 
N=6; % The number of rows +1
j=0;
n=length(route);
for i=1:n-1
    p=route(i); % First node in the route
    e=route(i+1); % Second node in the route 
    start_p=XY(p,:); % Coordinates of the first node 
    end_p=XY(e,:); %Cordniates of the second node 
    if i == 1 || i == n-1 % If the nodes are the first or the last node 
        path = manhattan_path(start_p, end_p); %Then its a manhattan path 
        kalman_filter(path)
%                 mp_pg(path); %generate manhattan path's robot trace
    elseif abs(p-e)==N      % its a straight line
        path=Line_way_p_gen(start_p,end_p,pts);
        if p>e
            % north to south node
            path=flip(path);
            %         else
            %             path=path;
        end
        kalman_filter(path)
    else                        % its a headland manuver
        % to identify the direction of the current turn
        
        if p<=11 && e>=2
            if p>e
                direction='southwest';
            elseif p<e
                direction='southeast';
            end
        elseif p>=12 && e<=21
            if p<e
                direction='northeast';
            elseif p>e
                direction='northwest';
            end
        end
        dist_n = abs(start_p(1) - end_p(1));
        
        if dist_n>=2*R_min      % if dist_n is greater than the 2*Rmin then its a pi turn
            path= pi_turn(R_min, dist_n,start_p, direction);
%             d2=(pi-2)*R_min;
%             d3=dist_n+d2;
%             final_position=pi_turn_pg(path,d3,start_position);
        else                % else omega turn
            path= omega_turn(R_min, dist_n,start_p, direction);
%             final_position=omega_turn_pg(path,start_position);
        end
    end
    
    length_path=length(path);
    for k=1:length_path
        total_path(j+k,1)=path(k,1);
        total_path(j+k,2)=path(k,2);
    end
    j=j+k;
    hold on
    grid
    scatter(path(1:end,1)', path(1:end,2)', 'b');pause(2)
end
