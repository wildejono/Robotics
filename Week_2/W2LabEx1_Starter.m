function W2LabEx1()
close all;
clc

%% Transform 1
T1 = se2(1, 2, 30*pi/180)
T1_h = trplot2(T1, 'frame', '1', 'color', 'b');
pause
%% Transform 2
T2 = se2(2, 1, 0)
hold on
T2_h = trplot2(T2, 'frame', '2', 'color', 'r');
pause
%% Transform 3 is T1*T2
T3 = T1*T2
T3_h =trplot2(T3, 'frame', '3', 'color', 'g');
pause
%% Transform 4 is T2*T1
T4 = T2*T1
T4_h =trplot2(T4, 'frame', '4', 'color', 'k');
pause

%% Axis
axis([0 5 0 5]);
axis equal;
grid on;
pause
%% Either quiz or lab exercise questions

%% What is the transform T5 between T3 and T4 (i.e. how can you transform T3 to be T4)
T5 = inv(T3) * T4; 
% Since
T3 * inv(T3) * T4 - T4 == 0

tranimate(T3,T4)
pause

%% Transform from transform to point
P = [3 ; 2 ];
plot_point(P, '*');

P1 = inv(T1) * [P; 1];
h2e( inv(T1) * e2h(P) )
% More compact
homtrans( inv(T1), P)

% In respect to T2
P2 = homtrans( inv(T2), P)
pause

%%
cla
%% Page 27
R = rotx(pi/2)
% trplot(R,'frame','1', 'rgb','rviz')
R_h = trplot(R,'frame','1', 'rgb','arrow')
axis equal;
grid on;
pause
% Incrementing by 1 deg 
for i=1:90
    try delete(R_h); end
    R = R * rotx(pi/180)
    R_h = trplot(R,'frame','1', 'rgb','arrow');
    drawnow();
    pause(0.01);
end
pause
%% Creating a new one from scratch
for i=1:2:90
    try delete(R_h); end
    R = rotx(i * pi/180)
    R_h = trplot(R,'frame','1', 'rgb','arrow');
    drawnow();
%     pause(0.01);
end
pause

%% Rotate back and forwards around each of the 3 axes
% Incrementing by 1 deg 
R = eye(3);
rotationAxis = 2;
for i = [[0:2:90],[89:-2:0]]
    try delete(R_h); end
    
    if rotationAxis == 1 % X axis rotation
        R = rotx(i * pi/180);
    elseif rotationAxis == 2 % Y axis rotation
        R = roty(i * pi/180);
    else % rotationAxis ==3 % Z axis rotation
        R = rotz(i * pi/180);
    end
    
    R_h = trplot(R,'frame','1', 'rgb','arrow');
    drawnow();
    pause(0.01);
end
pause

%% Rotate all 3 axis at the same time
cla;axis([-1,2,-1,2,-1,2])
R = eye(3);
for i = [[0:2:90],[89:-2:0]]
    try delete(R_h); end
    R = rotx(i * pi/180) * roty(i * pi/180) * rotz(i * pi/180);
    R_h = trplot(R,'frame','1', 'rgb','arrow');
    hold on;
    sumofAxes = R(:,1)+R(:,2)+R(:,3);
    plot3(sumofAxes(1),sumofAxes(2),sumofAxes(3),'*');
    drawnow();
    pause(0.01);
end
pause
%% Questions: Will the the plot of the sum of the axis vectors in the orientation be the same no matter the order
% The coloured point represent the sum of the 3 axis for different
% rotations. Which plot is which?
% a. red,green,blue,black = Rotations orders (x,y,z),(z,y,x),(x,z,y),(z,x,y)
% b. red,green,blue,black = Rotations orders (z,y,x),(x,z,y),(z,x,y),(x,y,z)
% c. red,green,blue,black = Rotations orders (x,z,y),(z,x,y),(x,y,z),(z,y,x)
% d. red,green,blue,black = Rotations orders (z,x,y),(x,y,z),(z,y,x),(x,z,y)
cla
for i = [[0:2:90],[89:-2:0]]
    try delete(R1_h); end
    try delete(R2_h); end
    try delete(R3_h); end
    try delete(R4_h); end
    R1 = rotx(i * pi/180) * roty(i * pi/180) * rotz(i * pi/180);
    R2 = rotz(i * pi/180) * roty(i * pi/180) * rotx(i * pi/180);
    R3 = rotx(i * pi/180) * rotz(i * pi/180) * roty(i * pi/180);
    R4 = rotz(i * pi/180) * rotx(i * pi/180) * roty(i * pi/180);
    R1_h = trplot(R1,'frame','1', 'rgb','arrow');
    R2_h = trplot(R2,'frame','2', 'rgb','arrow');
    R3_h = trplot(R3,'frame','3', 'rgb','arrow');
    R4_h = trplot(R4,'frame','4', 'rgb','arrow');
    hold on;
    sumofAxes1 = R1(:,1)+R1(:,2)+R1(:,3);
    sumofAxes2 = R2(:,1)+R2(:,2)+R2(:,3);
    sumofAxes3 = R3(:,1)+R3(:,2)+R3(:,3);
    sumofAxes4 = R4(:,1)+R4(:,2)+R4(:,3);
    plot3(sumofAxes1(1),sumofAxes1(2),sumofAxes1(3),'r*');
    plot3(sumofAxes2(1),sumofAxes2(2),sumofAxes2(3),'g*');
    plot3(sumofAxes3(1),sumofAxes3(2),sumofAxes3(3),'b*');
    plot3(sumofAxes4(1),sumofAxes4(2),sumofAxes4(3),'k*');
    drawnow();
end
keyboard