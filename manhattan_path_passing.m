
function [final_pose,final_pred,P_end]=manhattan_path_passing(path,counter,initial_pose,initial_pred,P_ini)
Ld=3;
% start_point=path(1,:);
end_point=path(end,:);
mid_point=[0 20];
t1=20;
if counter==1
    start_pos=[0;0;pi/2;0;0];
    q_true=start_pos;
    q_pred=[0;0;pi/2];
    P_ini=zeros(3);
    X=[end_point(1),end_point(2);mid_point(1),mid_point(2)];
    d2=pdist(X,'euclidean');
    t2=d2;
    T=t1+t2;
    [final_pose,final_pred,P_end]=closed_loop_control(path,Ld,T,q_true,q_pred,P_ini);
    
else 
    q_true=[initial_pose(1);initial_pose(2);initial_pose(3);0;0];
    q_pred=initial_pred;
    X=[end_point(1),end_point(2);mid_point(1),mid_point(2)];
    d2=pdist(X,'euclidean');
    t2=d2;
    T=t1+t2;
    [final_pose,final_pred,P_end]=closed_loop_control(path,Ld,T,q_true,q_pred,P_ini);
end 
     








% p_a=0;
% T=20;
% n_position=[-2.5 20];
% s_position=[-2.5 0];
% n=[-2.5 10];
% d1=15;
% 
% f=path(end,2);
% if end_point(2)==10
%     p_a=pi;
%     X=[end_point(1),end_point(2);n_position(1),n_position(2)];
%     d2=pdist(X,'euclidean');
%     T=d1;
%     
%     path_generator(T,path,start_pos,Ld);
% else
%     if f>0  %  its a cordinate in the northern node
%         p_a=pi/2;
%         X=[end_point(1),end_point(2);n_position(1),n_position(2)];
%         d2=pdist(X,'euclidean');
%         T=d1;
%         start_pos=[start_point(1) start_point(2) p_a];
%         y=path_generator(T,path(1:51,:),start_pos,Ld);
%         p_a=0;
%         T=d2;
%         start_pos=[n_position(1) n_position(2) p_a];
%         y=path_generator(T,path(51:end,:),start_pos,Ld);
%         
%     else   % its a cordinate in the southern node
%         p_a=-pi/2;
%         X=[end_point(1),end_point(2);s_position(1),s_position(2)];
%         d2=pdist(X,'euclidean');
%         T=d1;
%         start_pos=[start_point(1) start_point(2) p_a];
%         y=path_generator(T,path(1:51,:),start_pos,Ld);
%         p_a=0;
%         T=d2;
%         start_pos=[s_position(1) s_position(2) p_a];
%         y=path_generator(T,path(51:end,:),start_pos,Ld);
%     end
% end