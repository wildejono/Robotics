function DrawCircle3D(x,y,z,r,view)
%DRAWCIRCLE3D Summary of this function goes here
%   Detailed explanation goes here

hold on

ang=0:0.01:2*pi; 
    if view == "top"
        xp=r*cos(ang);
        yp=r*sin(ang);
        zp=zeros(length(xp));
        plot3(x+xp,y+yp,z+zp+0.1519,'Color','m');
    elseif view == "side"
        xp=r*cos(ang);
        yp=zeros(length(xp));
        zp=r*sin(ang);
        plot3(x+xp,y+yp,z+zp+0.1519,'Color','m');
    end
hold off
end

