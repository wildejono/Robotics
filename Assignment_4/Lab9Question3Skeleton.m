%% Robotics
% Lab 9 - Question 3 - Dynamic Torque

function Lab9Question3Skeleton()

close all
clear all
clc

mdl_puma560                                                                 % Load the puma560 model      
qZero = zeros(1,6);

%% Determine joint angles for the start and end pose
T1 = [[0 -1 0; 0 0 1; -1 0 0] [0;0.7;0];zeros(1,3) 1];                      % First pose to achieve
T2 = [[0 0 1;0 -1 0; 1 0 0] [0.5;0;0.6];zeros(1,3) 1];                      % Second pose to achieve

q1 = ... ;% Inverse kinematics
q2 = ... ;
time = ...	 % Total time to execute the motion
dt = 1/100;                                                                 % Set control frequency at 100Hz
steps = time/dt;                                                            % No. of steps along trajectory

%% Acceleration Control Questions
q = ...; % Generate joint trajectory between the two angles

qd = zeros(steps,6);                                                        % Array of joint velocities
qdd = nan(steps,6);                                                         % Array of joint accelerations
tau = nan(steps,6);                                                         % Array of joint torques
mass = -19.74;                                                                  % Payload mass (kg)
% 2.15 * 9.18 (force down) 21.38 - 215 = -193.62 > convert to mass to get
% -19.74
p560.payload(mass,[0.1;0;0]);                                               % Set payload mass in Puma 560 model: offset 0.1m in x-direction

for i = 1:steps-1
    qdd(i,:) = ... % Calculate joint acceleration to get to next set of joint angles
    M = ...  % Calculate inertia matrix at this pose
    C = ...  % Calculate coriolis MATRIX at this pose
    g = ... % Calculate gravity vector at this pose
    tau(i,:) = ... % Calculate the joint torque needed
    for j = 1:6
        if abs(tau(i,j)) > tau_max(j)                                       % Check if torque exceeds limits
            tau(i,j) = sign(tau(i,j))*tau_max(j);                           % Cap joint torque if above limits
        end
    end
    qdd(i,:) = ... % Re-calculate acceleration based on actual torque
    q(i+1,:) = ...  % Update joint angles based on actual acceleration
    qd(i+1,:) = ...  % Update the velocity for the next pose
end

t = 0:dt:(steps-1)*dt;                                                      % Generate time vector

%% Visulalisation and plotting of results

% Plot joint angles
figure(1)
for j = 1:6
    subplot(3,2,j)
    plot(t,q(:,j)','k','LineWidth',1);
    refline(0,p560.qlim(j,1));
    refline(0,p560.qlim(j,2));
    ylabel('Angle (rad)');
    box off
end

% Plot joint velocities
figure(2)
for j = 1:6
    subplot(3,2,j)
    plot(t,qd(:,j)*30/pi,'k','LineWidth',1);
    refline(0,0);
    ylabel('Velocity (RPM)');
    box off
end

% Plot joint acceleration
figure(3)
for j = 1:6
    subplot(3,2,j)
    plot(t,qdd(:,j),'k','LineWidth',1);
    ylabel('rad/s/s');
    refline(0,0)
    box off
end

% Plot joint torques
figure(4)
for j = 1:6
    subplot(3,2,j)
    plot(t,tau(:,j),'k','LineWidth',1);
    refline(0,tau_max(j));
    refline(0,-tau_max(j));
    ylabel('Nm');
    box off
end

% figure(6)
% p560.plot(q,'fps',steps)