% r = 3;
% dw = 2;
% pos = [0 0];
% direction = 'southwest';

function path = omega_turn(r, dw, pos, direction)

% figure(1); clf; hold on; axis equal;

    gamma  = acos(1-((2*r+dw)^2/(8*r^2)));
    alpha = (pi-gamma)/2;

    a1 = alpha:0.1:pi/2;
    x1 = r*sin(a1)-r;
    y1 = r*cos(a1);
    path = [x1' y1'];

    y_omega = sqrt(3*r^2 - r*dw - (dw^2)/4);
    x_omega = dw/2;

    a3 = -pi/2-gamma/2:0.1:pi/2+gamma/2;
    x3 = x_omega+r*sin(a3);
    y3 = y_omega+r*cos(a3);
    path = [fliplr(x3)' fliplr(y3)'; path];

    a2 = -alpha:-0.1:-pi/2;
    x2 = r+r*sin(a2)+dw;
    y2 = r*cos(a2);
    x2 = fliplr(x2);
    y2 = fliplr(y2);
    path = [x2' y2'; path];

    tt = transl2(pos(1),pos(2));

    if strcmp(direction, 'northeast') == 1
        tr = trot2(0);
        x_off = 0;
    elseif strcmp(direction, 'northwest') == 1
        tr = trot2(0);
        x_off = -dw;
        path = flipud(path);
    elseif strcmp(direction, 'southwest') == 1
        tr = trot2(pi);
        x_off = 0;
    elseif strcmp(direction, 'southeast') == 1
        tr = trot2(pi);
        x_off = dw;
        path = flipud(path);
    end
    t = tt*tr;

    path = [path(1:end,1)'; path(1:end,2)'; ones(1,length(path))];
    path = fliplr(path);
    path = t*path;
    path = [path(1, 1:end)'+x_off path(2, 1:end)'];
% scatter(path(1:end,1)', path(1:end,2)', 'b');
end