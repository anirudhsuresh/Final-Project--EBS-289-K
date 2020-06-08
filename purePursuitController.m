function [steer_angle, cross_track_error] = purePursuitController(q,  L, Ld, path)

% create a transformation matrix to convert the world frame 
% points to the tractors frame
t = transl2(q(1), q(2));
r = trot2(q(3));
transform = t * r;

% initialize the robot frame path and the distance matrices
robot_path = zeros(size(path,1),2);
d = zeros(size(path,1),2);

% convert the path into the robot frame and calculate distance
% between the robot and the path point
for i=1:1:size(path,1)
    p = [path(i,1); path(i,2); 1];
    point = pinv(transform) * p;
    robot_path(i,1:2)= point(1:2);
    d(i) = sqrt(point(1)^2 + point(2)^2);
end

% define variables for finding a valid point with minimum distance
min_val = Inf;
min_ind = 0;

% loop to find the minimum valid path point atleast Ld units away
for i=1:size(path,1)
    % skip if point is behind the robot 
    if robot_path(i,1) < 0
        continue
    end
    val = abs(Ld-d(i)); % make sure point is atleast Ld units away
    % update minimum value and index if necessary
    if val < min_val
        min_val = val;
        min_ind = i;
    end
end
   
% calculate the cross track error
[cte, ind] = min(d);
cross_track_error = cte(1);
if robot_path(ind(1),1) < 0
    cross_track_error = -cross_track_error;
end

% calculate curvature and steer angle
if min_ind > 0
    e = robot_path(min_ind,1:2);
else
    e = [0 0];
end
curvature = (2*e(2))/(Ld^2);
steer_angle = atan(curvature * L);

end