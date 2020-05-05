function Week4_Lecture_PointInAnotherCoordinateFrame(doExample)
if nargin < 1
    doExample = 5;
end

close all;

%% Example 0: Point In Global Coordinate Frame
% Global axes
globalTr = eye(4);
trplot(globalTr,'rgb','arrow');
hold on;

% Consider a 2D point (i.e. Z == 0)
point2d = [0.6,0.4,0];

% It has a transform in the global coodinate frame of
point2dTR = transl(point2d(1),point2d(2),point2d(3));

% This can be plotted with 
point2dTR_h = trplot(point2dTR);
axis([-2,2,-2,2]);
grid on;
view(2)

%-- How to put the point2d into the globalTr coordinate frame? --
% Method 1: Now in the global frame the point is
pointInGlobalFrame = (inv(eye(3)) * (point2d - globalTr(1:3,4)')')';
display(['pointInGlobalFrame = [',num2str(pointInGlobalFrame),']']);
% Method 2: Equally we could do it with the inverse of globalTr times point2dTR
point2dTRInGlobalFrame = inv(globalTr) * point2dTR

if 3 < doExample
    try delete(point2dTR_h); end
end

%% Example 1: Point in A New Coordinate Frame (with only translation & no rotation) 
if doExample == 1
    % Now consider (and plot) another coordinate frame
    newFrame = transl(1,1,0);
    newFrame_h = trplot(newFrame,'rgb','arrow');
    axis([-2,2,-2,2]);
    grid on;
    view(2)

 %-- How to put the point2d into the newFrame1 coordinate frame? --
    
    % Method 1: Where is point2d in in the newFrame1? Subtract the origin of
    % newFrame1 in the following way only works because there is no rotation
    pointInNewFrame = point2d - newFrame(1:3,4)';
    display(['Example ',num2str(doExample),': pointInNewFrame = [',num2str(pointInNewFrame),']']);
    
    % Method 2: Equally we could do it with the inverse of newFrame times point2dTR
    display(['Example ',num2str(doExample),': Method 2.']);
    point2dTRInNewFrame = inv(newFrame) * point2dTR
    keyboard
    try delete(newFrame_h); end

%% Example 2: Point in A New Coordinate Frame (with only rotation & no translation) 
elseif doExample == 2
    % Now consider (and plot) another coordinate frame
    newFrame = trotz(pi/4);
    newFrame_h = trplot(newFrame,'rgb','arrow');
    axis([-2,2,-2,2]);
    grid on;
    view(2)

 %-- How to put the point2d into the newFrame coordinate frame? --
    
    % Method 1: To get where point2d in in the newFrame we could make a negative version
    % of newFrame with a rotation in the oposite way (only possible because
    % there is not translation)
    newFrame_negative = se3(se2(0,0,-pi/4));
    pointInNewFrame = [newFrame_negative(1:3,1:3) * point2d']';
    display(['Example ',num2str(doExample),': Method 1. pointInNewFrame = [',num2str(pointInNewFrame),']']);
    
    % Method 2: We could extract the rotation part 
    R = newFrame(1:3,1:3);
    % and realise that to make R rotated to be Identity matrix (since I = inv(R) * R)
    eye(3) == inv(R) * R
    % then we can use the same method to rotate point2d
    pointInNewFrame = [inv(R) * point2d']';
    display(['Example ',num2str(doExample),': Method 2. pointInNewFrame = [',num2str(pointInNewFrame),']']);
    
    % Method 3: Equally we could do it with the full inverse of newFrame times point2dTR
    display(['Example ',num2str(doExample),': Method 3.']);
    point2dTRInNewFrame = inv(newFrame) * point2dTR
    keyboard
    try delete(newFrame_h); end

%% Example 3: Point in a New Coordinate Frame (with translation & rotation) 
elseif doExample == 3
    % Now consider (and plot) another coordinate frame
    newFrame = transl(1,1,0) * trotz(pi/4);
    newFrame_h = trplot(newFrame,'rgb','arrow');
    axis([-2,2,-2,2]);
    grid on;
    view(2)

 %-- How to put the point2d into the newFrame coordinate frame? --
    
    % Method1: We could translate back to the origin
    Tr = newFrame(1:3,4)';
    newFrame_atOrigin = newFrame - transl(Tr);
    %  and then extract the rotation part and rotate it back to the origin by
    %  realising that I = inv(R) * R  
    R = newFrame(1:3,1:3);
    eye(3) == inv(R) * R
    % Then we can use the same method to rotate point2d
    pointInNewFrame = [inv(R) * (point2d - Tr)']';
    display(['Example ',num2str(doExample),': Method 1. pointInNewFrame = [',num2str(pointInNewFrame),']']);
    
    % Method2: Equally we could do it with the full inverse of newFrame * point2dTR
    display(['Example ',num2str(doExample),': Method 2.']);
    point2dTRInNewFrame = inv(newFrame) * point2dTR
    keyboard
    try delete(newFrame_h); end

%% Example 4: 3D Point in a New Coordinate Frame (with translation & rotation) 
elseif doExample == 4    
    % Consider a 3D point
    point3d = [0.6,0.4,0.5];

    % It has a transform in the global coodinate frame of
    point3dTR = transl(point3d(1),point3d(2),point3d(3));

    % This can be plotted with 
    point3dTR_h = trplot(point3dTR);
    axis([-2,2,-2,2]);
    grid on;
    view(2)

    % Now consider (and plot) another coordinate frame 
    % translated by [1.1,1.2,1.3] 
    % then (rotated about X and Y axis by pi/4;
    newFrame = transl(1.1,1.2,1.3) * trotx(pi/4) * troty(pi/4);

    newFrame_h = trplot(newFrame,'rgb','arrow');
    axis([-3,3,-3,3,-3,3]);
    grid on;
    view(3)
    
 %-- How to put the point3d into the newFrame coordinate frame? --
    
    % Method1: We could translate back to the origin
    Tr = newFrame(1:3,4)';
    newFrame_atOrigin = newFrame - transl(Tr);
    %  and then extract the rotation part and rotate it back to the origin by
    %  realising that I = inv(R) * R  
    R = newFrame(1:3,1:3);
    eye(3) == inv(R) * R
    % Then we can use the same method to rotate point2d
    pointInNewFrame = [inv(R) * (point3d - Tr)']';
    display(['Example ',num2str(doExample),': Method 1. pointInNewFrame = [',num2str(pointInNewFrame),']']);
    
    % Method 2: Equally we could do it with the full inverse of newFrame * point2dTR
    display(['Example ',num2str(doExample),': Method 2.']);
    point3dTRInNewFrame = inv(newFrame) * point3dTR
    keyboard
    try delete(newFrame_h); end
    try delete(point3dTR_h); end
    
%% Example 5: Transform in a New Coordinate Frame (with translation & rotation) 
elseif doExample == 5
    try delete(point2dTR_h); end
    
    % Consider a transform in the global coodinate frame of
    transformMat = transl(0.6,0.4,0.5) * trotx(pi/3);

    % This can be plotted with 
    transformMat_h = trplot(transformMat);
    axis([-2,2,-2,2]);
    grid on;
    view(2)

    % Now consider (and plot) another coordinate frame 
    % translated by [1.1,1.2,1.3] 
    % then (rotated about X and Y axis by pi/4;
    newFrame = transl(1.1,1.2,1.3) * trotx(pi/4) * troty(pi/4);

    newFrame_h = trplot(newFrame,'rgb','arrow');
    axis([-3,3,-3,3,-3,3]);
    grid on;
    view(3)
    
 %-- How to put the point3d into the newFrame coordinate frame? --
    
    % Method1: We could translate back to the origin
    Tr = newFrame(1:3,4)';
    newFrame_atOrigin = newFrame - transl(Tr);
    %  and then extract the rotation part and rotate it back to the origin by
    %  realising that I = inv(R) * R  
    R = newFrame(1:3,1:3);
    eye(3) == inv(R) * R
    % Then we can use the same method to rotate point2d
    pointInNewFrame = [inv(R) * (transformMat(1:3,4)' - Tr)']';
    display(['Example ',num2str(doExample),': Method 1. pointInNewFrame = [',num2str(pointInNewFrame),']']);
    rotationInNewFrame = inv(R) * transformMat(1:3,1:3)    
    
    % Method 2: Equally we could do it with the full inverse of newFrame * transformMat
    display(['Example ',num2str(doExample),': Method 2.']);
    transformMatInNewFrame = inv(newFrame) * transformMat
    keyboard
    try delete(newFrame_h); end
    try delete(transform3d_h); end    
    
%% Example 6: Inverse in Matlab
elseif doExample == 6
    T = trotx(pi/4) * troty(pi/4) * transl(1.1,1.2,1.3);

    % Note also there are several ways of getting A into the T coordinate
    % frame with inverse times by another matrix in matlab 
    % 1) This is the one I use because it looks better: inv(T)*A
    % 2) This is the way that matlab recomends: T/A
    % 3) This is the way Jonathan does it: R = T(1:3,1:3);    t = T(1:3,4);
    % invT = [R' -R'*t; zeros(1,3) 1]; invT * A
    % Checkout the difference in value (note both should == identity matrix
    % 1) 
    eye(4) - inv(T) * T
    % 2)
    eye(4) - T / T
    % 3)
    R = T(1:3,1:3); t = T(1:3,4); invT = [R' -R'*t; zeros(1,3) 1]; 
    eye(4) - invT * T
end

