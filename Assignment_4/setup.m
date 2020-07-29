%% Summary
% This program was created for Assignment Task 4 of Robotics

%% Authors
% Jonathan Wilde - 12545606

%% Reference List
% =========================================================================

function [] = setup()
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%% Initialise MATLAB
close all
set(0,'DefaultFigureWindowStyle','docked')
clear
clc
% Mac run command
% run /Users/jonathanwilde/git/Robotics/rvctools/startup_rvc.m
% Windows run command
% run C:\Git\Robotics\rvctools\startup_rvc.m

%% Establish Workspace
workspace = [-1, 3, 0, 7, 0, 3];

%%  setting up environments
mdl_puma560

% Import a table (not important - cosmetic only)
    [f,v,data] = plyread('Objects\Table.ply','tri');
    % Get vertex count
    tableVertexCount = size(v,1);
    % Move center point to origin
    midPoint = sum(v)/tableVertexCount;
    tableVerts = v - repmat(midPoint,tableVertexCount,1);
    tablePose = eye(4);
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    hold on
    tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    hold off
    % Move table
    forwardTR = makehgtform('translate',[1.254,5.9,0.45]);
    randRotateTR = makehgtform('zrotate',0);
    tablePose = tablePose * forwardTR * randRotateTR;
    updatedPoints = [tablePose * [tableVerts,ones(tableVertexCount,1)]']';  

    % Now update the Vertices
    tableMesh_h.Vertices = updatedPoints(:,1:3);
    drawnow();
    
% Base location hard coded to SN values (12545606)
snX = 1.254;
snY = 5.606;
p560.base = transl([snX,snY,1])
p560.plot([0,0,0,0,0,0], 'workspace', workspace, 'scale', 0.3, 'nobase')


p560.teach()

% Adjust view
view(300,20);

%% Task 1.1: Determine the tool offset as shown in figure 1 of A4
p560.tool = transl(0, 0, 0.2) * troty(pi/4);

disp(p560.tool)

%% Task 1.2: Place the robot & drum. Display the following transforms:

% Transform of the robot base
disp(p560.base)
% Transform of the barrel origin

    % First import the barrel model
    [f,v,data] = plyread('Objects\Drum.ply','tri');
    % Get vertex count
    drumVertexCount = size(v,1);
    % Move center point to origin
    midPoint = sum(v)/drumVertexCount;
    drumVerts = v - repmat(midPoint,drumVertexCount,1);
    drumPose = eye(4);
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    hold on
    drumMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    hold off
    % Move table
    forwardTR = makehgtform('translate',[1.2,5,0.6]);
    randRotateTR = makehgtform('zrotate',pi/2);
    drumPose = drumPose * forwardTR * randRotateTR;
    updatedPoints = [drumPose * [drumVerts,ones(drumVertexCount,1)]']';  

    % Now update the Vertices
    drumMesh_h.Vertices = updatedPoints(:,1:3);
    drawnow();
    
    % Finally display the barrel base transform
    disp(drumPose)
    
% Transform between the robot base & the drum
base2drum = drumPose * inv(p560.base)

%% Task 1.3: Using inverse kinematic solver, show the development of 
%       joints for the puma that result in the tool end pointing toward 
%       the target corner of the white sheet

% Using the data tips function in MATLAB, I obtained the target corner
% location as:
corner1 = [1.139, 4.705, 0.868+0.3];
corner2 = [1.139, 4.895, 0.868+0.3];
tCorner = transl(corner1) * troty(pi);
qCorner = p560.ikcon(tCorner,[0,0,0,0,0,0]);

p560.plot(qCorner)

%% Task 1.4: Code and explain the following two scenarios

qZero = zeros(1,6);

%% Determine joint angles for the start and end pose
T1 = transl(corner1);                      % First pose to achieve
T2 = transl(corner2);                      % Second pose to achieve

q1 = p560.ikcon(T1, qZero);% Inverse kinematics
q2 = p560.ikcon(T2, qZero);

tau_max = [97.6 186.4 89.4 24.2 20.1 21.3]';

%% Moving slow enough to perform a successful blasting motion

time = 10	 % Total time to execute the motion
dt = 1/100;                                                                 % Set control frequency at 100Hz
steps = time/dt;                                                            % No. of steps along trajectory

% Acceleration Control Questions
% s = lspb(0,1,steps);                                                      % Generate trapezoidal velocity profile
% for i = 1:steps
%     q(i,:) = (1-s(i))*q1 + s(i)*q2;
% end
q = jtraj(q1,q2,steps);                                                     % Quintic polynomial profile

qd = zeros(steps,6);                                                        % Array of joint velocities
qdd = nan(steps,6);                                                         % Array of joint accelerations
tau = nan(steps,6);                                                         % Array of joint torques
mass = -19.74;                                                              % Payload mass (kg)
p560.payload(mass,[0;0;0.2]);                                               % Set payload mass in Puma 560 model: offset 0.1m in x-direction

for i = 1:steps-1
    qdd(i,:) = (1/dt)^2 * (q(i+1,:) - q(i,:) - dt*qd(i,:));                 % Calculate joint acceleration to get to next set of joint angles
    M = p560.inertia(q(i,:));                                               % Calculate inertia matrix at this pose
    C = p560.coriolis(q(i,:),qd(i,:));                                      % Calculate coriolis matrix at this pose
    g = p560.gravload(q(i,:));                                              % Calculate gravity vector at this pose
    tau(i,:) = (M*qdd(i,:)' + C*qd(i,:)' + g')';                            % Calculate the joint torque needed
    for j = 1:6
        if abs(tau(i,j)) > tau_max(j)                                       % Check if torque exceeds limits
            tau(i,j) = sign(tau(i,j))*tau_max(j);                           % Cap joint torque if above limits
        end
    end
    qdd(i,:) = (inv(M)*(tau(i,:)' - C*qd(i,:)' - g'))';                     % Re-calculate acceleration based on actual torque
    q(i+1,:) = q(i,:) + dt*qd(i,:) + dt^2*qdd(i,:);                         % Update joint angles based on actual acceleration
    qd(i+1,:) = qd(i,:) + dt*qdd(i,:);                                      % Update the velocity for the next pose
end

t = 0:dt:(steps-1)*dt;                                                      % Generate time vector                                                   % Generate time vector

% Visulalisation and plotting of results

% Plot joint angles
figure(2)
for j = 1:6
    subplot(3,2,j)
    plot(t,q(:,j)','k','LineWidth',1);
    refline(0,p560.qlim(j,1));
    refline(0,p560.qlim(j,2));
    ylabel('Angle (rad)');
    box off
end

% Plot joint velocities
figure(3)
for j = 1:6
    subplot(3,2,j)
    plot(t,qd(:,j)*30/pi,'k','LineWidth',1);
    refline(0,0);
    ylabel('Velocity (RPM)');
    box off
end

% Plot joint acceleration
figure(4)
for j = 1:6
    subplot(3,2,j)
    plot(t,qdd(:,j),'k','LineWidth',1);
    ylabel('rad/s/s');
    refline(0,0)
    box off
end

% Plot joint torques
figure(5)
for j = 1:6
    subplot(3,2,j)
    plot(t,tau(:,j),'k','LineWidth',1);
    refline(0,tau_max(j));
    refline(0,-tau_max(j));
    ylabel('Nm');
    box off
end

figure(1)
p560.plot(q,'fps',steps)

end