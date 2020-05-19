%% Initial Setup
close all
set(0,'DefaultFigureWindowStyle','docked')
clear
clc
run C:\Users\Jay\Data\Robotics\rvctools\startup_rvc.m

%% Using PUMA 560, check various values of Q for singularity
mdl_puma560
% (Answer is the one closest to 0)
q = [0 2.1677 -2.7332 0 -0.9425 0];
jacobian = p560.jacob0(q);
measureofmanip = sqrt(det(jacobian(1:2,:)*jacobian(1:2,:)'))
p560.plot(q)

%%
% Load a model of a 3-link planar robot with mdl_planar3 (note robot is
% called 'p3') Which of these poses is closest to a singularity
clear
clc
mdl_planar3
q = [0 1.5708 -1.5708];
%q = [0.5 0.5 0.5];

jacobian = p3.jacob0(q);
measureOfManip = sqrt(det(jacobian(1:2,:)*jacobian(1:2,:)'))
%% 
%Get RectangularPrism.m and isCollision.m. Create mdl_planar3 and and a 
% rectangular prism with [v,f,fn] = RectangularPrism([2,-1,-1], [3,1,1]).
% Create a 50 step trajectory with jtrej from q1=[pi/3,0,0] to
% q2=[-pi/3,0,0] and use "result=isCollision(q3,q,f,v,fn) to determine the
% first pose in the trajectory that is in collision?

clear
clc
mdl_planar3
stepCount = 50;
q1 = [pi/3,0,0];
q2 = [-pi/3,0,0];

trej = jtraj(q1,q2,stepCount);

[v,f,fn] = RectangularPrism([2,-1,-1], [3,1,1]);

for i = 1:numel(trej)
    
    q = trej(i,:)
    
    result = IsCollision(p3,q,f,v,fn);
    
    if(result == 1)
        r = trej(i,:)
        break 
    end
end

%%
%Ellipsoid
clear
clc

[X,Y] = meshgrid(-10:1:10,-10:1:10);
Z = X;
[x,y,z] = ellipsoid(3,2,1,1,2,3);
surf(X,Y,Z)
hold on
surf(x,y,z)

%% Create Camera
% cam=CentralCamera('focal',0.08,'pixel',10e-5,'resolution',[1024
% 1024],'centre',[512 512],'name','UR10camera'); mounted on a UR10 with a
% current joint configuration of [1.6; -1;-1.2;-0.5;0;0]; in radians
% given the four coordinates of the image target,
% pStar=(600 300 300 600; 300 300 600 600)
% and the location of the 3D points in 
% P=[2,2,2,2; -0.3,0.3,0.3,-0.3; 1.3,1.3,0.7,0.7];
% Calculate the feature error for the current configuration
% e=pStar-uv

P=[2,2,2,2; -0.3,0.3,0.3,-0.3; 1.3,1.3,0.7,0.7];

pStar = [600 300 300 600; 300 300 600 600];

cam = CentralCamera('focal',0.08,'pixel',10e-5,'resolution',[1024 1024], 'centre',[512 512],'name','UR10camera');
r = UR10();

q0 =  [1.6; -1; -1.2; -0.5; 0; 0];
Tc0 = r.model.fkine(q0);
cam.T = Tc0;
uv = cam.plot(P);
e = pStar-uv