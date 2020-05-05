%% Intialise script
clear all
clc
clf
set(0,'DefaultFigureWindowStyle','docked')
run /Users/jonathanwilde/MATLAB-Drive/RVC1/rvctools/startup_rvc.m

%% Question 
% Create a baxter robot with mdl_baxter
% given the arm models are called "left" and "Right", determine the
% distance between the ***"base" of the left arm and the end effector*** of the
% right arm when qRight = [-9*pi/10,0,0,4*pi/9,0,0,0]

mdl_baxter
qRight = [0,0,0,0,0,0,0]; %change this
%qLeft = [pi/6,0,0,0,0,0,-3*pi/2];

left_base = left.base
r_endeffect = right.fkine(qRight)

x1 = left.base(1,4)
y1 = left.base(2,4)
z1 = left.base(3,4)

x2 = r_endeffect(1,4)
y2 = r_endeffect(2,4)
z2 = r_endeffect(3,4)

D = sqrt((x2-x1)^2+(y2-y1)^2+(z2-z1)^2)

%% Question
% create a baxter robot with mdl_baxter. given the arm models are called
% 'left' and 'right', determine the distance between ***the two ende
% effectors**
% when qleft = [3*pi/10 0 0 0 0 0 2*pi/10] qRight =
% [4*pi/10 0 0 -2*pi/5 0 0 0]
mdl_baxter
qLeft = [5*pi/3,0,0,0,8*pi/2,0,0]; %change this
qRight = [7*pi/4,0,0,0,0,0,-3*pi/20]; %change this


left = left.fkine(qLeft)
right = right.fkine(qRight)

x1 = left(1,4)
y1 = left(2,4)
z1 = left(3,4)


x2 = right(1,4)
y2 = right(2,4)
z2 = right(3,4)

D = sqrt((x2-x1)^2+(y2-y1)^2+(z2-z1)^2)
%% Question
% Create a puma 560 with mdl_puma560
% Assume it is bolted onto a table 0.8m off the ground with q =
% [45,45,45,0,45,0] degrees. we have a ball whose center is defined by a
% global transform transl[0.7,0,0.4]*trotx(pi/2) what is the balls position
% with respect to the end-effectors coordinate frame

mdl_puma560
p560.base = transl(0, 0, 0.8); %base location of the puma 560
ball = transl(0.5,0,0.6) * trotx(pi/2) %global transform of the ball
q = [0, 45, 0, 0, 45, 0] %workspace vector

endTool_world = p560.fkine(deg2rad(q)) %end effector tool with respect to the world ref coordinates ref coordinates
end_ball = inv(endTool_world)*ball %end effector tool with respect to the ball

%% Question
% Create a puma 560 with mdl_puma560
% Assume it is bolted to a ***table*** which is 0.8m off the ground. Given the
% arm model is called "p560", determine the distance from the end effector
% to the floor when q = [0,pi/10,0,0,0,0]
mdl_puma560
q = [0, 45, 0, 0, 45, 0]; %change this
p560.base = transl(0, 0, 0.8); %change this
endTool_world = p560.fkine(q)

endtool_floor = endTool_world(3,4)

%% Question
% Create a puma 560 with mdl_puma560
% Assume it is bolted to the ***floor***. Given the
% arm model is called "p560", determine the distance from the end effector
% to the floor when q = [0,pi/10,0,0,0,0]
mdl_puma560
p560.base = transl(0, 0, 0.4); %change this
q = [0,pi/10,0,0,0,0]; %change this
endTool_world = p560.fkine(q) %change this
endtool_floor = endTool_world(3,4)

%% Create a puma 560 with mdl_puma560
% part 1: will the following be true
%1: Will the following be true fkine(p560,ikine(p560,fkine(p560,[-pi/4,0,0,0,0,0][-pi/4,0,0,0,0,0]))==fkine(p560,[-pi/4,0,0,0,0,0])
%2: Will the following be true
%fkine(p560,ikine(p560,fkine(p560,[pi/4,0,0,0,0,0]),[pi/4,0,0,0,0,0]))==fkine(p560,[pi/4,0,0,0,0,0])

mdl_puma560

part1_a = fkine(p560,ikine(p560,fkine(p560,[pi/4,0,0,0,0,0])))
part1_b = fkine(p560,[pi/4,0,0,0,0,0])

part2_a = fkine(p560,ikine(p560,fkine(p560,[pi/4,0,0,0,0,0]),[pi/4,0,0,0,0,0]))
part2_b = fkine(p560,[pi/4,0,0,0,0,0])

part1 = part1_a==part1_b
part2 = part2_a==part2_b

%%
mdl_puma560
p560.base = transl(0, 0, 0); %base location of the puma 560
%q = [p560.qlim(:,1)+(p560.qlim(:,2)-p560.qlim(:,1))/2]
q = pi/8*ones(1,6)
p560.jacob0(q)

%% create a puma 560 with mdl_puma560 on the floor
% move to the min of its range(ie q = p560.qlim(:,1):) then compute the
% jacobian at this pose(ie p560.jacob0(q)). based upon this matrix what cna
% you say about the robot at this pose?
mdl_puma560


%% Create a hyper redundant planar manipulator with mdl_hyper2d
% which pose would be in self collision? (hint: plot the arm at each of the
% joint states)
mdl_hyper2d
% CORRECT
q = [pi, pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, -pi/2, -pi/3, -pi/3];
h2d.plot(q)





