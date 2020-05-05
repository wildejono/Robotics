%% create the robot
clear all
clf
clc

mdl_planar2

%% define the workspace, and plot

q = zeros(1,2);
p2.plot(q,'workspace',[-1 2.8 -2.5 2.5 -0.05 2],'scale',0.5);

p2.teach;

%% move the arm using the teach gui, and display Jacobian

qCurrent = p2.getpos;
while(1)
    if ~isequal(qCurrent, p2.getpos)
        qCurrent = p2.getpos;
        disp(['The Jacobian for the current joint angles ([', ...
            num2str(p2.getpos),']) is:']);
        disp(p2.jacob0(p2.getpos));
       
    end
    pause(0.5)
end

%% plot the manipulability ellipsoid (ellipse for this case)

eePose = p2.fkine(p2.getpos);
J = p2.jacob0(p2.getpos);
try delete(ellipsePlot); end;
ellipsePlot = plot_ellipse(J(1:2,:)*J(1:2,:)', eePose(1:2,4), ...
    'fillcolor', 'b', 'alpha', 0.6);

%% sample through joint 2 angles between limits

p2.plot([0,-pi/2]);
% measureOfManip = sqrt(det(J(1:2,:)*J(1:2,:)'));
% threshold = 0.3;

for i = p2.qlim(2,1):pi/180:p2.qlim(2,2)
% for i = -pi/2:pi/180:pi/2
    
    eePose = p2.fkine(p2.getpos);
    J = p2.jacob0(p2.getpos);
    measureOfManip = sqrt(det(J(1:2,:)*J(1:2,:)'));
%     if measureOfManip < threshold
%         disp('Robot stopped: approaching singularity');
%         break;
%     end
    p2.plot([0,i]);
    try delete(ellipsePlot); end;
    ellipsePlot = plot_ellipse(J(1:2,:)*J(1:2,:)', eePose(1:2,4), ...
    'fillcolor', 'b', 'alpha', 0.6);

end

