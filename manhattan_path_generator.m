function path_m = manhattan_path_generator(start, finish)
path_count = 1;

if start(1) < finish(1) && start(2) < finish(2)
    for i=start(2):0.2:finish(2)
        path_m(path_count,1:2) = [start(1),i];
        path_count = path_count+1;
    end
    for i=start(1):0.2:finish(1)
        path_m(path_count,1:2) = [i,path_m(end,2)];
        path_count = path_count+1;
    end
end

if start(1) > finish(1) && start(2) > finish(2)
    for i=finish(2):0.2:start(2)
        path_m(path_count,1:2) = [finish(1),i];
        path_count = path_count+1;
    end
    for i=finish(1):0.2:start(1)
        path_m(path_count,1:2) = [i,path_m(end,2)];
        path_count = path_count+1;
    end
    path_m = flipud(path_m);
end

if start(1) < finish(1) && start(2) > finish(2)
    for i=start(2):-0.2:finish(2)
        path_m(path_count,1:2) = [start(1),i];
        path_count = path_count+1;
    end
    for i=start(1):0.2:finish(1)
        path_m(path_count,1:2) = [i,path_m(end,2)];
        path_count = path_count+1;
    end
end

if start(1) > finish(1) && start(2) < finish(2)
    temp = start;
    start = finish;
    finish = temp;
    for i=start(2):-0.2:finish(2)
        path_m(path_count,1:2) = [start(1),i];
        path_count = path_count+1;
    end
    for i=start(1):0.2:finish(1)
        path_m(path_count,1:2) = [i,path_m(end,2)];
        path_count = path_count+1;
    end
    path_m = flipud(path_m);
end

% path_m
% scatter(path_m(:,1)', path_m(:,2)', 'b');