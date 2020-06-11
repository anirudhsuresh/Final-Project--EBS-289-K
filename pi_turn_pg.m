function [final_pose]=pi_turn_pg(path,initial_pose,d3)

T=d3;
Ld=3;
initial_pose=[initial_pose(1);initial_pose(2);initial_pose(3);0;0];
[final_pose]=closed_loop_control(path,Ld,T,initial_pose);