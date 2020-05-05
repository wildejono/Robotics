%% Initialise Matlab
InitialiseMatlab()
disp('Initialisation of MATLAB complete...')

%% Task 1 - Publish two UR3s to a workspaceCoord
% TABLE LOCATION VALUES
tableOffset = [0, 0.26, -0.77];
% ASSESSMENT BASE COORDINATES
base1Coord = [-0.23, 0.23, 0];
base2Coord = [0.23, 0.23, 0];
base1 = transl(base1Coord);
base2 = transl(base2Coord);
% ROBOT HOME POSE VALUES
holdCoord = [0, 0.4, 0.2];
homeQ = [-pi/2 -pi/2 pi/2 -pi/2 -pi/2 0];
% Set up all accompanying workspace assets
pcbOffset = [0, 0.46, 0];
topOffset = [0.42, 0.46, 0.01];
botOffset = [-0.42, 0.46, 0.02];
disp('Workspace will now be set up')
[pcbVerts,pcbVertCount,topHousingVerts,topHousingVertCount,...
    botHousingVerts,botHousingVertCount,...
    pcbMesh,topMesh,botMesh] = PlotWorkspace(pcbOffset,topOffset,botOffset,tableOffset);

robot1InitialPose = zeros(1,6);
robot2InitialPose = zeros(1,6);

robot1 = UR3(base1);
robot2 = UR3(base2);

% view([37.5 30]);
view([210 30]);

disp('Workspace has been set up!')

%% Sequence of manufacturing events
disp('Manufacturing sequence will begin now...')
disp('Robot 1 moving to grab PCB')
passT2 = transl(pcbOffset + [0, 0, 0.2]) * troty(pi);
robot1.MoveRobot(passT2);

passT2 = transl(pcbOffset) * troty(pi);
robot1.MoveRobot(passT2);

disp('Robot 1 has grabbed PCB')
disp('Robot 2 moving to grab top housing')
passT2 = transl(topOffset + [0, 0, 0.2]) * troty(pi);
robot2.MoveRobot(passT2);

passT2 = transl(topOffset) * troty(pi);
robot2.MoveRobot(passT2);

disp('Robot 2 has grabbed top housing') 
disp('Robot 1 moving PCB to join location')
passT2 = transl(holdCoord + [-0.05, 0, 0]) * troty(pi/2);
robot1.MoveRobotGrab(passT2,pcbMesh,pcbVerts,pcbVertCount,0);

disp('Robot 2 moving top housing to join location')
passT2 = transl([0.2, 0.6, 0.15]) * troty(-pi/2);
robot2.MoveRobotGrab(passT2,topMesh,topHousingVerts,topHousingVertCount,0);

disp('Robots are working to join PCB and top housing')
passT2 = transl(holdCoord + [0.05, 0, 0]) * troty(-pi/2);
robot2.MoveRobotGrab(passT2,topMesh,topHousingVerts,topHousingVertCount,0);

passT2 = transl(holdCoord + [0, 0, 0]) * troty(-pi/2);
robot2.MoveRobotGrab(passT2,topMesh,topHousingVerts,topHousingVertCount,0);

passT2 = transl(holdCoord + [-0.025, 0, 0]) * troty(pi/2);
robot1.MoveRobotGrab(passT2,pcbMesh,pcbVerts,pcbVertCount,0); 

passT2 = transl(holdCoord + [-0.05, 0, 0]) * troty(pi/2);
robot1.MoveRobot(passT2);

disp('Robot 1 moving to grab bottom housing')
passT2 = transl([-0.2, 0.6, 0.15]) * troty(pi);
robot1.MoveRobot(passT2);

passT2 = transl(botOffset + [0, 0, 0.2]) * troty(pi);
robot1.MoveRobot(passT2);

passT2 = transl(botOffset) * troty(pi);
robot1.MoveRobot(passT2);

disp('Robot 1 has grabbed bottom housing')
disp('Robot 1 moving bottom housing to join location')
passT2 = transl([-0.2, 0.6, 0.15]) * troty(pi);
robot1.MoveRobotGrab(passT2,botMesh,botHousingVerts,botHousingVertCount,1);

passT2 = transl(holdCoord + [-0.1, 0, 0]) * troty(pi/2);
robot1.MoveRobotGrab(passT2,botMesh,botHousingVerts,botHousingVertCount,1);

passT2 = transl(holdCoord + [-0.05, 0, 0]) * troty(pi/2);
robot1.MoveRobotGrab(passT2,botMesh,botHousingVerts,botHousingVertCount,1);

disp('Robot 1 is now homing, tasks complete!')
passT2 = transl(holdCoord + [-0.2, 0, 0]) * troty(pi/2);
robot1.MoveRobot(passT2);

passT2 = transl([-0.2, 0.45, 0.15]) * troty(pi);
robot1.MoveRobot(passT2);

robot1.MoveHome();

disp('Robot 2 is depositing assembly to dropoff location')
passT2 = transl([0.5, -0.1, 0.4]) * trotx(pi/2);
robot2.MoveRobotGrabAll(passT2,pcbMesh,topMesh,botMesh,...
    pcbVerts,topHousingVerts,botHousingVerts,pcbVertCount,...
    topHousingVertCount,botHousingVertCount,0,0,0);

passT2 = transl([0.6, 0.18, 0.3]) * trotx(pi/2);
robot2.MoveRobotGrabAll(passT2,pcbMesh,topMesh,botMesh,...
    pcbVerts,topHousingVerts,botHousingVerts,pcbVertCount,...
    topHousingVertCount,botHousingVertCount,0,0,0);

passT2 = transl([0.6, 0.18, 0.1]) * trotx(pi);
robot2.MoveRobotGrabAll(passT2,pcbMesh,topMesh,botMesh,...
    pcbVerts,topHousingVerts,botHousingVerts,pcbVertCount,...
    topHousingVertCount,botHousingVertCount,0,0,0);

disp('Robot 2 is now homing, tasks complete!')
passT2 = transl([0.5, -0.1, 0.4]);
robot2.MoveRobot(passT2);

robot2.MoveHome();

disp('Complete!')

%% Task 2(a) - Calculate maximum reach of the arm
% Obtain transform of end effector at UR3 maximum reach for XAXIS
robot1StraightQ = [0, 0, 0, -pi/2, pi/2, 0];
robot2StraightQ = [0, 0, 0, -pi/2, pi/2, 0];

robot1StraightTr = robot1.model.fkine(robot1StraightQ);
robot2StraightTr = robot2.model.fkine(robot2StraightQ);

robot1XRadius = abs((robot1StraightTr(1,4) - base1(1,4)));
robot2XRadius = abs((robot2StraightTr(1,4) - base2(1,4)));

disp('Robot 1 maximum reach in X axis is:')
disp(robot1XRadius)
disp('Robot 2 maximum reach in X axis is:')
disp(robot2XRadius)

% Obtain transform of end effector at UR3 maximum reach for ZAXIS
robot1StraightQ = [0, -pi/2, 0, -pi/2, pi/2, 0];
robot2StraightQ = [0, -pi/2, 0, -pi/2, pi/2, 0];

robot1StraightTr = robot1.model.fkine(robot1StraightQ);
robot2StraightTr = robot2.model.fkine(robot2StraightQ);

robot1ZRadius = abs((robot1StraightTr(3,4) - (base1(3,4) + 0.1519)));
robot2ZRadius = abs((robot2StraightTr(3,4) - (base2(3,4) + 0.1519)));

% Below is the maximum reach along ZAXIS from the reference of the robot
% base

robot1ZMaxReach = abs((robot1StraightTr(3,4) - base1(3,4)));
robot2ZMaxReach = abs((robot2StraightTr(3,4) - base2(3,4)));

disp('Robot 1 maximum reach in Z axis is:')
disp(robot1ZMaxReach)
disp('Robot 2 maximum reach in Z axis is:')
disp(robot2ZMaxReach)

% Draw circles representing the radii
DrawCircle3D(base1(1,4),base1(2,4),base1(3,4),robot1XRadius,"top")
DrawCircle3D(base1(1,4),base1(2,4),base1(3,4),robot1ZRadius,"side")

DrawCircle3D(base2(1,4),base2(2,4),base2(3,4),robot2XRadius,"top")
DrawCircle3D(base2(1,4),base2(2,4),base2(3,4),robot2ZRadius,"side")

disp('Maximum reach of each arm has been calculated!')

%% Task 2(b) - Demonstrate the approximate volume in meters(cubed)
stepRads = deg2rad(10);

robot1QLim = robot1.model.qlim;
robot2QLim = robot2.model.qlim;

robot1PointCloudeSize = prod(floor((robot1QLim(1:2,2)-robot1QLim(1:2,1))/stepRads + 1));
robot2PointCloudeSize = prod(floor((robot2QLim(1:2,2)-robot2QLim(1:2,1))/stepRads + 1));

robot1PointCloud = zeros(robot1PointCloudeSize,3);
robot2PointCloud = zeros(robot2PointCloudeSize,3);

counter = 1;
tic

for q1 = robot1QLim(1,1):stepRads:robot1QLim(1,2)
    for q2 = robot1QLim(2,1):stepRads:robot1QLim(2,2)
        qContainer = [q1,q2,0,-pi/2,pi/2,0];
        tr = robot1.model.fkine(qContainer);
        robot1PointCloud(counter,:) = tr(1:3,4)';
        counter = counter + 1;
    end
end

counter = 1;
tic

for q1 = robot2QLim(1,1):stepRads:robot2QLim(1,2)
    for q2 = robot2QLim(2,1):stepRads:robot2QLim(2,2)
        qContainer = [q1,q2,0,-pi/2,pi/2,0];
        tr = robot2.model.fkine(qContainer);
        robot2PointCloud(counter,:) = tr(1:3,4)';
        counter = counter + 1;
    end
end

hold on
plot3(robot1PointCloud(:,1),robot1PointCloud(:,2),robot1PointCloud(:,3),'r*');
plot3(robot2PointCloud(:,1),robot2PointCloud(:,2),robot2PointCloud(:,3),'b*');
hold off

areaRobot1 = 4*pi*((robot1XRadius)^2);
areaRobot2 = 4*pi*((robot2XRadius)^2);

disp('Robot 1 working volume is:')
disp(areaRobot1)
disp('Robot 2 working volume is:')
disp(areaRobot1)

disp('Working area of each arm demonstrated & calculated!')

%% Blank working space for robot movement
% This will take a target point, and move the robot to the desired location
% This will also return the final position matrix of the end effector
desiredPoint = [0.2, 0.6, 0.2];
customT2 = transl(desiredPoint) * troty(pi);
robot2.MoveRobot(customT2);

%% End of script
disp('Script has finished!')