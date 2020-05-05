%% Initialise Matlab
InitialiseMatlab()

%% Create a Puma model

workspace = [-1 1 -1 1 -1 1];
mdl_puma560

% ADJUST the q positions of the robot joints in accordance with the
% question
q = [90, 30, -80, 0, 50, 0];
q = deg2rad(q);
p560.teach(q);
hold on

endTool_world = p560.fkine(q);

% ADJUST the balls translation and rotation in accordance with the question
ball = transl(0.5,0.1,0.6) * trotx(pi/2);

end_ball = inv(endTool_world)*ball;

x = end_ball(1,4);
y = end_ball(2,4);
z = end_ball(3,4);

plot_sphere([x,y,z],0.05,'blue',0.1)

answer = [x y z]

%% Given an end effector determine joints of puma

mdl_puma560

% ADJUST this based on question
EndEffector = transl(0.6,0.1,0.1);

% Question output
q = p560.ikine(EndEffector,qn,[1,1,1,0,0,0])
% UNCOMMENT IF YOU WANT TO APPLY MASK
% qq = p560.ikine(EndEffector,q,[1,1,1,1,1,0])
% qq = p560.ikine(EndEffector,[1,1,1,1,1,0])

%% Ray cast

mdl_puma560

q = [pi/12,0,-pi/2,0,0,0];
wall = 1.8;

transform = p560.fkine(q)
EEtransform = transform * transl(0,0,1);

ans = LinePlaneIntersection([1,0,0],[wall,0,1],transform(1:3,4)',EEtransform(1:3,4)')

%% 5 DOF robot

L1 = Link('d',0,'a',1,'alpha',0,'offset',0);
L2 = Link('d',0,'a',1,'alpha',0,'offset',0);
L3 = Link('d',0,'a',1,'alpha',0,'offset',0);
L4 = Link('d',0,'a',1,'alpha',0,'offset',0);
L5 = Link('d',0,'a',1,'alpha',0,'offset',0);

q = [30, -60, 45,-30,0];
U = SerialLink([L1 L2 L3 L4 L5]);   
M = U.fkine(deg2rad(q));
x = M(1,4);
y = M(2,4);
z = M(3,4);

answer = [x y z]

%% 3D Link

mdl_3link3d
q = [-pi/9,0,0];
wall = 4.1;
tf = R3.fkine(q)
% [ans, f] = LinePlaneIntersection([1,0,0],[wall,0,1],[0,0,1],tf(1:3,4)')
ans = LinePlaneIntersection([1,0,0],[wall,0,1],[0,0,1],tf(1:3,4)')

%% Puma sensor distance (sub in to sensorxyz)

mdl_puma560
q = [0,40,-80,0,45,0];
EE = p560.fkine(deg2rad(q));
EEx = EE(1,4);
EEy = EE(2,4);
EEz = EE(3,4);

Sensorx = 1;
Sensory = 0;
Sensorz = 1;

% distance = sqrt((Sensorx-EEx)^2 + (Sensory-EEy)^2 + (Sensorz-EEz)^2 )

Sensory = sqrt((distance)^2 - (Sensorx-EEx)^2 - (Sensorz-EEz)^2) + EEy

%% MAX ABS velocity

q1 = [pi/10,pi/7,pi/5,pi/3,pi/4,pi/6];
q2 = [-pi/10,-pi/7,-pi/5,-pi/3,-pi/4,-pi/6];
steps = 150

qMatrix = jtraj(q1,q2,steps);

s = lspb(0,1,steps);                                             	% First, create the scalar function
qMatrix = nan(steps,6);                                             % Create memory allocation for variables
    for i = 1:steps
        qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;                   	% Generate interpolated joint angles
    end
        
velocity = zeros(steps,6);
acceleration  = zeros(steps,6);
for i = 2:steps
    velocity(i,:) = qMatrix(i,:) - qMatrix(i-1,:);                          % Evaluate relative joint velocity
    acceleration(i,:) = velocity(i,:) - velocity(i-1,:);                    % Evaluate relative acceleration
end

disp(velocity)

velocity = abs(velocity)

disp(velocity)

maxV = max(velocity)

disp(maxV)