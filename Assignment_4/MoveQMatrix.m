function [] = MoveQMatrix(Robot_Arm,qMatrix,Objects,Environment,stepsize)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

if ~exist('stepsize')
    stepsize = 1;
end

for qStep = 1:stepsize:size(qMatrix,1)
    q = qMatrix(qStep,:);    
    Robot_Arm.NOJOY = true;
    % Check joy, if 0 joystick is not initialised
    if Robot_Arm.NOJOY == true
        % Do nothing and skip the ESTOP check
    else
        try estop = Robot_Arm.checkJoy();
        catch estop = 1;
        end
        
        while estop == 1
            disp('ESTOP DETECTED - HALTING ALL ACTIONS')
            pause(1)
            try estop = Robot_Arm.checkJoy();
            catch estop = 1;
            end
        end
    end
    
    if size(Environment,2) == 0
        colResult = false;
    else
        colResult = true;
    end
    
    while colResult == true
        for i = 1:size(Environment,2)
            [movePoints, moveFaces, moveNormals] = Environment(i).getPLYData();

            colResult = IsCollision(Robot_Arm.model,qMatrix,moveFaces,...
                movePoints,moveNormals,'1');
            
            if colResult == true
                disp('Collision detected - Trying again in 2 seconds...')
                pause(1)
                disp('Collision detected - Trying again in 1 seconds...')
                pause(1)
                disp('Attempting to move again')
                break
            end
        end
    end
    
    Robot_Arm.model.animate(q);
    if isempty(Objects) == 0
        newBase = Robot_Arm.model.fkine(q);
    end
    for i = 1:size(Objects,2)
        Objects(i).model.base = newBase * trotx(Objects(i).rot);
        Objects(i).model.animate(0);
    end
    pause(0.01);
end

end

