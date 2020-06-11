function path=Line_way_p_gen(start_p,end_p,pts)
        m=(end_p(2)-start_p(2))/(end_p(1)-start_p(1)); %gradient 
        if m==Inf  %vertical line
            xx(1:pts)=start_p(1);
            yy(1:pts)=linspace(start_p(2),end_p(2),pts);
        elseif m==-Inf % vertical line
            xx(1:pts)=start_p(1);
            yy(1:pts)=linspace(end_p(2),start_p(2),pts);
%         elseif m==0 %horizontal line
%             xx(1:pts)=linspace(start_p(1),end_p(1),pts);
%             yy(1:pts)=start_p(2);
%         else %if (endp(1)-startp(1))~=0
%             xx=linspace(start_p(1),end_p(1),pts);
%             yy=m*(xx-start_p(1))+start_p(2);
        end
        path=[xx',yy'];
end