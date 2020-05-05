%% Robotics
% Lab 5 - Question 1 - UFO Blasting
function [  ] = Lab5SolutionQuestion1Skeleton(  )
clf

%% 1.2 Add UFO Fleet
ufoFleet = UFOFleet(10);

%% 1.3 Add blasting robot
blasterRobot = SchunkUTSv2_0();
plot3d(blasterRobot,zeros(1,6));
endEffectorTr = blasterRobot.fkine(zeros(1,6));

blasterRobot.delay = 0;

%% 1.4 Plot a “blast” cone 
[X,Y,Z] = cylinder([0,0.1],6);
Z = Z * 10;
updatedConePoints = [endEffectorTr * [X(:),Y(:),Z(:),ones(numel(X),1)]']';
conePointsSize = size(X);
cone_h = surf(reshape(updatedConePoints(:,1),conePointsSize) ...
             ,reshape(updatedConePoints(:,2),conePointsSize) ...
             ,reshape(updatedConePoints(:,3),conePointsSize));
view(3);

%% 1.5 Plot the point score board
currentScore = 0;
scoreZ = ufoFleet.workspaceDimensions(end)*1.2;
text_h = text(0, 0, scoreZ,sprintf('Score: 0 after 0 seconds'), 'FontSize', 10, 'Color', [.6 .2 .6]);

%% 1.6 “while loop” to iteratively call your function
% Start timer
tic

% Go through iterations of randomly move UFOs, then move robot. Check for
% hits and update score and timer
while ~isempty(find(0 < ufoFleet.healthRemaining,1))
    ufoFleet.PlotSingleRandomStep();    
    
    % Get the goal joint state
    goalJointState = GetGoalJointState(blasterRobot,ufoFleet);

    % Fix goal pose back to a small step away from the min/max joint limits
    fixIndexMin = goalJointState' < blasterRobot.qlim(:,1);
    goalJointState(fixIndexMin) = blasterRobot.qlim(fixIndexMin,1) + 10*pi/180;
    fixIndexMax = blasterRobot.qlim(:,2) < goalJointState';
    goalJointState(fixIndexMax) = blasterRobot.qlim(fixIndexMax,2) - 10*pi/180;
        
    % Get a trajectory
    jointTrajectory = jtraj(blasterRobot.getpos(),goalJointState,8);
    for armMoveIndex = 1:size(jointTrajectory,1)
        animate(blasterRobot,jointTrajectory(armMoveIndex,:));
        
        endEffectorTr = blasterRobot.fkine(jointTrajectory(armMoveIndex,:));        
        updatedConePoints = [endEffectorTr * [X(:),Y(:),Z(:),ones(numel(X),1)]']';        
        set(cone_h,'XData',reshape(updatedConePoints(:,1),conePointsSize) ...
                  ,'YData',reshape(updatedConePoints(:,2),conePointsSize) ...
                  ,'ZData',reshape(updatedConePoints(:,3),conePointsSize));
              
        coneEnds = [cone_h.XData(2,:)', cone_h.YData(2,:)', cone_h.ZData(2,:)'];        
        ufoHitIndex = CheckIntersections(endEffectorTr,coneEnds,ufoFleet);
        ufoFleet.SetHit(ufoHitIndex);
        currentScore = currentScore + length(ufoHitIndex);
        
        text_h.String = sprintf(['Score: ',num2str(currentScore),' after ',num2str(toc),' seconds']);
        axis([-6,6,-6,6,0,10]);
        % Only plot every 3rd to make it faster
        if mod(armMoveIndex,3) == 0
            drawnow();
        end
    end
    drawnow();
end
display(['Finished in ',num2str(toc),' seconds with score of ',num2str(currentScore)]);
% Log results to file

end

%% GetGoalJointState
% Given the current blasterRobot and ufoFleet determine the next pose and
% thus joint state to move to 
function goalJointState = GetGoalJointState(blasterRobot,ufoFleet)
    %% Solution 1: random guess facing upwards
    if solutionToUse == 1
        goalJointState = blasterRobot.getpos() + (rand(1,6)-0.5) * 20*pi/180;
        endEffectorTr = blasterRobot.fkine(goalJointState);

        % Ensure the Z component of the Z axis is positive (pointing upwards),
        % and the Z component of the point is above 1 (approx mid height)
        while endEffectorTr(3,3) < 0.1 || endEffectorTr(3,4) < 1
            goalJointState = blasterRobot.getpos() + (rand(1,6)-0.5) * 20*pi/180;
            endEffectorTr = blasterRobot.fkine(goalJointState);
            display('trying again');
        end
    
    end

    %% Implement a better solution here and comment out the above solution   
end



    