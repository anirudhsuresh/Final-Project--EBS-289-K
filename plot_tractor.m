% This function allows us to plot the transformation matrix 

function plot_tractor(T, color)
    global tractor;
    new_t = T * tractor;
    
    plot_poly(new_t, 'fillcolor', color, 'edgecolor', color, 'alpha', 1, 'EdgeAlpha', 1);
%     pause(0.1)

end
