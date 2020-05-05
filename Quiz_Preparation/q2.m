clc
%%
%Q1 baxter base to left end effector distance (page4)
mdl_baxter

left_q = [ 0 0 0 0 0 0 0];
right_q = [-9*pi/10 0 0 4*pi/9 0 0 0];


left_base = left.base
r_endeffect = right.fkine(right_q)

x1 = left.base(1,4)
y1 = left.base(2,4)
z1 = left.base(3,4)


x2 = r_endeffect(1,4)
y2 = r_endeffect(2,4)
z2 = r_endeffect(3,4)

D = sqrt((x2-x1)^2+(y2-y1)^2+(z2-z1)^2)


%%
%Q6  puma (page1)
mdl_puma560

p560.base = transl(0, 0, 0.8); %base location of the puma560
ball = transl(0.7,0,0.4)*trotx(pi/2) %global transform of the ball
q = [45 45 45 0 45 0] %workspace vector

endTool_world = p560.fkine(deg2rad(q)) %end effector tool with respect to the world ref coordinates
end_ball = inv(endTool_world)*ball %end effector tool with respect to the ball
%%
%Q9(page4)
mdl_puma560
% p560.base = transl(0, 0, 0);
q = [0 pi/10 0 0 0 0];

endEffector_world = p560.fkine(q)
% x1 = p560.base(1,4)
% y1 = p560.base(2,4)
% z1 = p560.base(3,4)
% 
% x2 = endEffector_world(1,4)
% y2 = endEffector_world(2,4)
% z2 = endEffector_world(3,4)
% % D = sqrt((x2-x1)^2+(y2-y1)^2+(z2-z1)^2)

%%
%Q distance between 2 end effectors with baxter 
mdl_baxter

qLeft = [3*pi/10 0 0 0 0 0 2*pi/10];
qRight = [4*pi/10 0 0 -2*pi/5 0 0 0];

left = left.fkine(qLeft)
right = right.fkine(qRight)

x1 = left(1,4)
y1 = left(2,4)
z1 = left(3,4)


x2 = right(1,4)
y2 = right(2,4)
z2 = right(3,4)

D = sqrt((x2-x1)^2+(y2-y1)^2+(z2-z1)^2)



%%
%Q10 (page4)
mdl_puma560
q = [0 pi/10 0 0 0 0];
p560.base = transl(0, 0, 0.8);
endTool_world = p560.fkine(q)

endtool_floor = endTool_world(3,4)



