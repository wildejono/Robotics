function Lab4_3DOF_DrawLine()

%% Standard start
clear all
clc
clf

%% Drawing with 3DOF Arm

% Make the arm model
L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);

robot = SerialLink([L1 L2 L3],'name','myRobot');

% Robote the base around the Y axis so Z faces down ways
robot.base = troty(pi);

% Prepare workplace, plot the robot and prepare to teach
q = zeros(1,3);
robot.plot(q,'workspace',[-2 2 -2 2 -0.05 2],'scale',0.5);
robot.teach;

% Solution for end effector
newQ = robot.ikine(transl(-0.75,-0.5,0),q,[1,1,0,0,0,0]);

% Plot new joint state
robot.animate(newQ)
robot.fkine(newQ)

% Looping to draw a line with the robot arm
start = [-0.75 -0.5 0];
finish = [-0.75 0.5 0];

steps = 60;

qMatrix = zeros(steps,3);
transform = transl((finish - start) / steps)
nextPose = transl(start)
for i = 1:steps
   nextPose = nextPose * transform
   newQ = robot.ikine(nextPose,newQ,[1,1,0,0,0,0]);
   robot.animate(newQ);
   tool = robot.fkine(newQ);
   qMatrix(i,:) = transl(tool);
   drawnow();
end