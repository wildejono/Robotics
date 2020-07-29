function [qMatrix,steps] = MoveWObjects(Robot_Arm,GoalPose,Objects,Environment)
%MoveWObjects Summary of this function goes here
%   Takes in a robot arm, a pose and a matrix of objects and plots them
%   Returning the robot endeffector pose and the objects poses in a same
%   size matrix
% ObjectsPoses = zeros;
zoffset = -0.0;
% animate 1
initQ = Robot_Arm.model.getpos .* [1,0,0,0,0,0];
initQ(2) = pi/2;
if size(GoalPose,2) == 6
    goalQ = GoalPose;
else
%     goalQ = Robot_Arm.model.ikcon(GoalPose * trotx(pi) * transl(0,0,zoffset),initQ);
    goalQ = Robot_Arm.model.ikcon(GoalPose * trotx(pi) * transl(0,0,zoffset),Robot_Arm.model.getpos);
end
jointTrajectory = jtraj(Robot_Arm.model.getpos(), goalQ,30);

% MoveQMatrix(Robot_Arm,jointTrajectory,Objects,Environment);
qMatrix = jointTrajectory;
steps = 30;
% if size(GoalPose,2) == 6
%     return
% else
%     goalQ = Robot_Arm.model.ikcon(GoalPose * trotx(pi),Robot_Arm.model.getpos);
% end
% jointTrajectory = jtraj(Robot_Arm.model.getpos(), goalQ,30);
% 
% MoveQMatrix(Robot_Arm,jointTrajectory,Objects,Environment);

% RobotEndPose = Robot_Arm.model.fkine(Robot_Arm.model.getpos);
end

