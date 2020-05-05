%% Robotics
% Lab 6 
%% Quesiton 3: Joint Interpolation vs Resolved Motion Rate Control
function [  ] = Lab6Question3Skeleton( )
close all;

%% Quesiton 3
% Joint Interpolation

% 3.1
steps = 50;
mdl_planar2;                                  % Load 2-Link Planar Robot

% 3.2
T1 = [eye(3) [1.5 1 0]'; zeros(1,3) 1];       % First pose
T2 = [eye(3) [1.5 -1 0]'; zeros(1,3) 1];      % Second pose

% 3.3
M = [1 1 zeros(1,4)];                         % Masking Matrix
q1 = p2.ikine(T1,[0 0],M);                    % Solve for joint angles
q2 = p2.ikine(T2,[0 0],M);                    % Solve for joint angles

% 3.4
qMatrix = ...;
p2.plot(qMatrix,'trail','r-');

% 3.5: Resolved Motion Rate Control
steps = 50;

% 3.6
x1 = [1.5 1]';
x2 = [1.5 -1]';
deltaT = ...;                                        % Discrete time step

% 3.7
x = zeros(2,steps);
s = lspb(0,1,steps);                                 % Create interpolation scalar
for i = 1:steps
    x(:,i) = x1*(1-s(i)) + s(i)*x2;                  % Create trajectory in x-y plane
end

% 3.8
qMatrix = nan(steps,2);

% 3.9
qMatrix(1,:) = p2.ikine(T1,[0 0],M);                 % Solve for joint angles

% 3.10
for i = 1:steps-1
    xdot = ...;                             % Calculate velocity at discrete time step
    J = p2.jacob0(qMatrix(i,:));            % Get the Jacobian at the current state
    J = J(1:2,:);                           % Take only first 2 rows
    qdot = ...;                             % Solve velocitities via RMRC
    qMatrix(i+1,:) = ...;                   % Update next joint state
end

p2.plot(qMatrix,'trail','r-');

end
