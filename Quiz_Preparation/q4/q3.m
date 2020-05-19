%% PUMA560 singularity 
% clc

mdl_puma560
q = [0 2.1677 -2.7332 0 -0.9425 0];
jacobian = p560.jacob0(q);
measureOfManip = sqrt(det(jacobian(1:2,:)*jacobian(1:2,:)'))



%% Planar3

% clc
mdl_planar3
q = [0.7854 -0.7854 0.7854];

jacobian = p3.jacob0(q);
measureOfManip = sqrt(det(jacobian(1:2,:)*jacobian(1:2,:)'))

%% Ellipsoid
[X,Y] = meshgrid(-10:1:10,-10:1:10);
Z = X;
[x,y,z] = ellipsoid(3,2,1,1,2,3);
surf(X,Y,Z)
hold on
surf(x,y,z)
