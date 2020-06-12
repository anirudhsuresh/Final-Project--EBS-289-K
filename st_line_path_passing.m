function [final_pose,final_pred,P_end]=st_line_path_passing(path,initial_pose,initial_pred,counter,P_ini)
T=20;
Ld=3;
if counter==1
    q_true=[initial_pose(1);initial_pose(2);pi/2;0;0];
else 
	q_true=[initial_pose(1);initial_pose(2);initial_pose(3);0;0];
end 
q_pred=initial_pred;
[final_pose,final_pred,P_end]=closed_loop_control(path,Ld,T,q_true,q_pred,P_ini);


