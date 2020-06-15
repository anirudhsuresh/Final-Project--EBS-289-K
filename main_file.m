%--------------
% This is the main_function file in the final project 
generateNursery();
optimal_route_generator() % runs the node sequnce generator function
scan_trees()
load('occupancy_grid.mat')
detect_trees(occupancy_grid, 'finaloutput.txt')