function Lab2Solution()
% close all;
clf
clc

input('Press enter to begin')

%% Exercise 1: Animate transform (Quad copter flying)
%1.1
tranimate(eye(4),transl([0,0,10]),'fps',25)
% CUSTOM WORKSPACE
trStart = transl([0,0,0])
%trEnd = transl([0.8,0,1.5])
%trEnd = trotz(-90 * pi/180)
trEnd = transl([1.5,0.8,0]) * trotx(180 * pi/180)
%trEnd = troty(180 * pi/180) * transl([0.8,0,1.5])
%1.2
trStart = transl([0,0,0])
trEnd = transl([0,1.5,0.8]) * trotz(180 * pi/180)
%trEnd = trotx(180 * pi/180) * transl([1.5,0.8,0])
tranimate(trStart,trEnd,'fps',25);
%1.3
trStart = trEnd;
trEnd = transl([0,2,10]) * trotx(-30 * pi/180);
tranimate(trStart,trEnd,'fps',25);
%1.4
trStart = trEnd;
trEnd = transl([0,2,10]);
tranimate(trStart,trEnd,'fps',25);
%1.5
trStart = trEnd;
trEnd = transl([0,2,10]) * troty(30 * pi/180);
tranimate(trStart,trEnd,'fps',25);
%1.6
trStart = trEnd;
trEnd = transl([2,2,10]) * troty(30 * pi/180);
tranimate(trStart,trEnd,'fps',25);
%1.7
trStart = trEnd;
trEnd = transl([2,2,10]);
tranimate(trStart,trEnd);
%1.8
trStart = trEnd;
trEnd = transl([2,2,0]);
tranimate(trStart,trEnd,'fps',25);
%1.9 
%is already done above

input('Finished exercises 1, press enter to continue')

%% Exercise 2 (done before 1 so it can keep this all in one file)
% 2.1-2.2 
% Follow the instructions on the PDF

% 2.3 Create an instance of the cow herd with default parameters
cowHerd = RobotCows();
% 2.4 Check how many cow there are
cowHerd.cowCount
% 2.5 Plot on single iteration of the random step movement
cowHerd.PlotSingleRandomStep

input('Finished exercise 2.5, press enter to continue')

% 2.6 Clear then create another instance with 10 cows
clf;
clear all;
try delete(cowHerd); end;
cowHerd = RobotCows(10);
% 2.7 Test many random steps
numSteps=100;
delay=0.01;
cowHerd.TestPlotManyStep(numSteps,delay);
% 2.8 Query the location of the 2nd cow 
cowHerd.cow{2}.base

input('Finished exercises 2, press enter to continue');

%% Exercise 3
% 3.1 Create a cow herd with more than 2 cows.
cowHerd = RobotCows(3);
% 3.2 Plot the transform of the UAV starting at the origin
uavTR{1} = eye(4);
trplot(uavTR{1})
% 3.3 Determine the transform between the UAV and each of the cows 
for cowIndex = 1:cowHerd.cowCount
    display(['At trajectoryStep ',num2str(1),' the UAV TR to cow ',num2str(cowIndex),' is ']);
    display(num2str(inv(uavTR{1}) * cowHerd.cow{cowIndex}.base));
end  
cowHerd.PlotSingleRandomStep();

% 3.4-3.5 Fly through Exercise 1, at each time the UAV moves to a goal, the
% cows move randomly once, then determine the transform between the UAV and
% all the cows 
uavTR{2} = transl([0,0,10]);
uavTR{3} = transl([0,0,10]) * trotx(-30 * pi/180);
uavTR{4} = transl([0,2,10]) * trotx(-30 * pi/180);
uavTR{5} = transl([0,2,10]);
uavTR{6} = transl([0,2,10]) * troty(30 * pi/180);
uavTR{7} = transl([2,2,10]) * troty(30 * pi/180);
uavTR{8} = transl([2,2,10]);
uavTR{9} = transl([2,2,0]);

for trajectoryStep = 1:size(uavTR,2)-1
    tranimate(uavTR{trajectoryStep},uavTR{trajectoryStep+1},'fps',25)
    cowHerd.PlotSingleRandomStep();
    for cowIndex = 1:cowHerd.cowCount
        display(['At trajectoryStep ',num2str(trajectoryStep+1),' the UAV TR to cow ',num2str(cowIndex),' is ']);
        display(num2str(inv(uavTR{trajectoryStep+1}) * cowHerd.cow{cowIndex}.base));
    end    
end

input('Finished exercise 3.5, press enter to continue')

% 3.6 Create a cow herd with 1 cow and move your drone so that at each step the cow moves stay 5 meters above it but directly overhead
clf;
clear all;
clc;
cowHerd = RobotCows(1);
uavTRStart = eye(4);
uavTRGoal = transl([0,0,5]);
tranimate(uavTRStart,uavTRGoal,'fps',25)
% Go through 10 steps (no specified but this is arbitary)
for i = 1:100
    cowHerd.PlotSingleRandomStep();
    uavTRStart = uavTRGoal;
    uavTRGoal = cowHerd.cow{1}.base * transl(0,0,5);
    tranimate(uavTRStart,uavTRGoal,'fps',100);
end

input('Finished exercises 3, press enter to continue')

%% Exercise 4 Derive the DH parameters for the simple 3 link manipulator provided. 
% Use these to generate a  model of the manipulator using the Robot Toolbox in MATLAB 

clf;
clear all;
clc
% 4.1 and 4.2: Define the DH Parameters to create the Kinematic model
L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])

robot = SerialLink([L1 L2 L3],'name','myRobot')                     % Generate the model

workspace = [-4 4 -4 4 -4 4];                                       % Set the size of the workspace when drawing the robot

scale = 0.5;

q = zeros(1,3);                                                     % Create a vector of initial joint angles

robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot

% 4.3 Manually play around with the robot
robot.teach;                                                        % Open a menu to move the robot manually

% 4.4 Get the current joint angles based on the position in the model
q = robot.getpos();  

% 4.5 Get the joint limits
robot.qlim 