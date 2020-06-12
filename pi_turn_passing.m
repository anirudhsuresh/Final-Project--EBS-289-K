function [final_pose,final_pred,P_end]=pi_turn_passing(path,initial_pose,initial_pred,d3,P_ini)
T=d3;
Ld=3;
q_true=[initial_pose(1);initial_pose(2);initial_pose(3);0;0];
q_pred=initial_pred;
[final_pose,final_pred,P_end]=closed_loop_control(path,Ld,T,q_true,q_pred,P_ini);