% This function allows us to create a array having vertices to a tractor 
% In our case it is a trangle 

function build_tractor()
    global tractor;
    scaling = 0.2;
    tractor = [0 0 2*scaling; scaling -scaling 0; 1 1 1];
end
