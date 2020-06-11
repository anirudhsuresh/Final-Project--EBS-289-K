%   figure(1); clf; hold on; axis equal;
%  R = 3; % update this value as well
%  d_nodes = 7.5; % this should eb the distance between the midpoints of the two lines
%  start_p = [0 0];
%  direction = 'southwest';
% % 
% %   path=pi_turn1(r, dw, pos, direction)
% 
%  
function path = pi_turn(R, d_nodes, start_p, direction)

    a = 0:-0.1:-pi/2;
    x = R+R*sin(a);
    y = R*cos(a);
    x = fliplr(x);
    y = fliplr(y);
    path = [x' y'];

    circle_end = path(end, 1:2);
    path_count = length(path(1:end,1));

    for i = 0:0.3:d_nodes-2*R
        path(path_count, 1:2) = [circle_end(1)+i, circle_end(2)];
        path_count = path_count + 1;
    end

    a = 0:0.1:pi/2;
    x = R*sin(a)+path(end,1);
    y = R*cos(a);
    path = [path; x' y'];

    tt = transl2(start_p(1),start_p(2));

    if strcmp(direction, 'northeast') == 1
        tr = trot2(0);
        x_off = 0;
        path = flipud(path);
    elseif strcmp(direction, 'northwest') == 1
        tr = trot2(0);
        x_off = -d_nodes;
    elseif strcmp(direction, 'southwest') == 1
        tr = trot2(pi);
        x_off = 0;
        path = flipud(path);
    elseif strcmp(direction, 'southeast') == 1
        tr = trot2(pi);
        x_off = d_nodes;
    end
    t = tt*tr;

    path = [path(1:end,1)'; path(1:end,2)'; ones(1,length(path))];
    path = fliplr(path);
    path = t*path;
    path = [path(1, 1:end)'+x_off path(2, 1:end)'];
%    scatter(path(1:end,1)', path(1:end,2)', 'b'); 
end
