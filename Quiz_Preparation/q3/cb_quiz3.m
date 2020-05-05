%% Collision checking
close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear
workSpace = [-6 6 -6 6 0 6];

mdl_3link3d
q =  [pi/2,0,0];

wall = 6;

tf = R3.fkine(q)

% ans = LinePlaneIntersection([1,0,0],[wall,0,1],[0,0,1],tf(1:3,4)')
ans = LinePlaneIntersection([-1,0,0],[wall,0,1],[0,0,1],tf(1:3,4)')  % returns intersectionPoint, check

R3.plot(q,'workspace',workSpace);

%% 5DOF Planar - set alpha to zero
close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear
workSpace = [-1 1 -1 1 -1 1];

L(1) = Link('d',0,'a',1,'alpha',0,'qlim',deg2rad([-360 360])); 
L(2) = Link('d',0,'a',1,'alpha',0,'qlim',deg2rad([-360 360])); 
L(3) = Link('d',0,'a',1,'alpha',0,'qlim',deg2rad([-360 360])); 
L(4) = Link('d',0,'a',1,'alpha',0,'qlim',deg2rad([-360 360])); 
L(5) = Link('d',0,'a',1,'alpha',0,'qlim',deg2rad([-360 360]));

robot = SerialLink(L);
q = deg2rad([45,-45,45,-45,0]);
robot.fkine(q)
robot.teach();

%% Distanse sense distance to puma endeffector
close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear
workSpace = [-1 1 -1 1 -1 1];

mdl_puma560 

q = deg2rad([0, 45, -80, 0, 45, 0] );
tr = p560.fkine(q);

sLx = 0;
sLy = -0.15;
sLz = 0.647;

effx = tr(1,4);
effy = tr(2,4);
effz = tr(3,4);

distance = sqrt((sLx-effx)^2 +(sLy-effy)^2+(sLz-effz)^2)

p560.plot(q,'workspace',workSpace);
hold on
plot_sphere([sLx, sLy, sLz], 0.05,'blue', 0.1);
p560.teach()

%% Lab Assignment 1

%% Point in Puma End Effector Coord frame
close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear
workSpace = [-1 1 -1 1 -1 1];
mdl_puma560 

q = deg2rad([0,0,0,0,0,0]);
p560.teach(q);
hold on;
% ball = transl(0.5,0.1,0.6) * trotx(pi/2)
x = 0.5;
y = -0.150;
z = 0.432;
ball = transl(x,y,z)
plot_sphere([x,y,z], 0.05,'blue', 0.1);

ef2ball = inv(p560.fkine(q))* ball
%% Puma Ikine
close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear
workSpace = [-1 1 -1 1 -1 1];

mdl_puma560 

eef = transl(0.6,0.1,0.1);

q1 = p560.ikine(eef)

q2 = p560.ikine(eef,repelem(0,6),[1 0 0 0 0 0])

p560.teach(q1);
p560.fkine(p560.getpos())

%% Puma distance to Wall along Z
