function Lab4_Puma560()
%% Initial Setup
close all
set(0,'DefaultFigureWindowStyle','docked')
clear
clc
run C:\Users\Jay\Data\Robotics\rvctools\startup_rvc.m

mdl_puma560
% Determine which of the joint poses result in a singularity
% (Answer is the one closest to 0)
q = [0 2.1677 -2.7332 0 -0.9425 0];
jacobian = p560.jacob0(q);
measureofmanip = sqrt(det(jacobian(1:2,:)*jacobian(1:2,:)'))
p560.plot(q)

%% Next Steps

T1 = transl(0.5,-0.4,0.5);

q1 = p560.ikine(T1);

T2 = transl(0.5,0.4,0.1);

q2 = p560.ikine(T2);

steps = 50;

% Method one
%qMatrix = jtraj(q1,q2,steps)

% Method two
s = lspb(0,1,steps); qMatrix = nan(steps,6); 
for i = 1:steps
    qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
end

figure(1)
p560.plot(qMatrix,'trail','r-')

velocity = zeros(steps,6); acceleration = zeros(steps,6);
for i = 2:steps
    velocity(i,:) = qMatrix(i,:) - qMatrix(i-1,:);
    acceleration(i,:) = velocity(i,:) - velocity(i-1,:);
end

p560.teach()