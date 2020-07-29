classdef p560model < handle% setup and move the UR3 robot, as well as log its transforms
    properties
        model;
        currentJoints;
        location;
        workspace;
        plyData;
        name;
        pointCloud;
        qValueMatrix;
        Max_Reach;
        Max_Vol;
        robotFunctions;
    end
    
    methods
        function self = D6Model(name,workspace,location)
            %if run with no args then run demo
            if nargin == 0
                close all
                set(0,'DefaultFigureWindowStyle','docked')                
                
                self.workspace = [-2,2,-2,2,0,2];
                self.getRobot("test");
                self.currentJoints = deg2rad([90,90,90,0,0,0]);
                self.homeJoints = deg2rad([90,90,90,0,0,0]);
                self.model.base = transl(0,0,0);
                self.location = transl(0,0,0);
                self.name = "test";
                self.PlotAndColour();
                
                self.model.teach()
                
            else
                self.workspace = workspace;
                self.getRobot(name);
                self.currentJoints = zeros(1,6);
                self.homeJoints = deg2rad([90,90,90,0,0,0]);
                self.model.base = location;
                self.location = location;
                self.name = name;
            
                self.PlotAndColour();
            end
        end
        
        function selfupdate(q1,q2,q3,q4,q5,q6)
            self.model.animate([q1,q2,q3,q4,q5,q6]);
        end
        
        function home(self)
            self.model.animate(self.homeJoints);
        end
        
        function [pose] = getPose(self)
            pose = self.model.fkine(self.model.getpos);
        end
        
        function [pointCloud] = GeneratePointCloud(self, stepSize)
            stepRads = deg2rad(stepSize);
            qlim = self.model.qlim;
            % Don't need to worry about joint 6
            pointCloudeSize = prod(floor((qlim(1:(self.model.n - 1),2)-qlim(1:(self.model.n - 1),1))/stepRads + 1));
            qValueMatrix = zeros(pointCloudeSize,self.model.n);
            pointCloud = ones(pointCloudeSize,3) .* self.model.base(1:3,4)';
            counter = 1;
            count = 1;
            tic
            
            for q1 = qlim(1,1):stepRads:qlim(1,2)
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    for q3 = qlim(3,1):stepRads:qlim(3,2)
                        for q4 = qlim(4,1):stepRads:qlim(4,2)
                            for q5 = qlim(5,1):stepRads:qlim(5,2)
                                % Don't need to worry about joint 6, just assume it=0
                                q6 = 0;
                                %                               for q6 = qlim(6,1):stepRads:qlim(6,2)
                                q = [q1,q2,q3,q4,q5,q6];
                                if(self.withinBounds(q) == 1)
                                    qValueMatrix(counter,:) = q;
                                    tr = self.model.fkine(q);
                                    pointCloud(counter,:) = tr(1:3,4)';
                                    counter = counter + 1;
                                end
                                count = count + 1;
                                if mod(count/pointCloudeSize * 100,1) == 0
                                    display(['After ',num2str(toc),' seconds, completed ',num2str(count/pointCloudeSize * 100),'% of poses']);
                                end
                                %                               end
                            end
                        end
                    end
                end
            end
            %             ReducePointCloud(pointCloud);
            
            for i = 1 : pointCloudeSize
                if pointCloud(i,:) == self.model.base(1:3,4)'
                    pointCloud = pointCloud(1:i-1,:);
                    break
                end
            end
            self.pointCloud = pointCloud;
            self.qValueMatrix = qValueMatrix;
            save('PcloudReduced','pointCloud','qValueMatrix');
        end
        
        function [pCloud] = LoadPointCloud(self)
            temp = load('PcloudReduced','pointCloud','qValueMatrix');
            self.pointCloud = temp.pointCloud;
            self.qValueMatrix = temp.qValueMatrix;
            pCloud = self.pointCloud;
        end
        
        function ReducePointCloud(self, pointCloud)
            [pCloudSize, c] = size(pointCloud);
            
            for i = 1 : pCloudSize
                if pointCloud(i,:) == self.model.base(1:3,4)'
                    pointCloud = pointCloud(1:i-1,:);
                    break
                end
            end
            self.pointCloud = pointCloud;
            save('PcloudReduced','pointCloud');
        end
        
        function Vol = MaxRobotVolume(self)
            if isempty(self.pointCloud)
                Vol = 0;
                display('no point cloud generated for this model yet');
            else
                [k, Vol] = convhull(self.pointCloud);
                self.Max_Vol = Vol;
            end
        end
        
        function [Reach, index] = MaxRobotReach(self)
            if isempty(self.pointCloud)
                Reach = 0;
                display('no point cloud generated for this model yet');
            else
                [r,c] = size(self.pointCloud);
                output = zeros(r,1);
                
                for i=1:r
                    output(i,1) = norm(self.model.base(1:3,4)' - self.pointCloud(i,:));
                end
                
                [Reach,index] = max(output);
                self.Max_Reach = Reach;
            end
        end
        
        function PlotAndColour(self)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Models/N6-PLY/J',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            
            % Display robot
            self.model.plot3d(self.currentJoints,'workspace',self.workspace,'floorlevel', 0);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight;
            end
            
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        
        function getRobot(self, name) % Setup Robot Parameters
            pause(0.001);
            % This begins @ joint 1
            % joint values from
            % https://epson.com/For-Work/Robots/6-Axis/Flexion-N6-Compact-6-Axis-Robots---1000mm/p/RN6-A10SS73SS
%               J1 (Turning): +/-180 deg
%               J2 (Lower Arm): +/-180 deg
%               J3 (Upper Arm): +/-180 deg
%               J4 (Wrist Roll): +/-200 deg
%               J5 (Wrist Bend): +/-125 deg
%               J6 (Wrist Twist): +/-360 deg
            L1 = Link('d',0.867,'a',0.15,'alpha',pi/2,'qlim',deg2rad([-360 360]));
            L2 = Link('d',0,'a',0.350,'alpha',0,'qlim',deg2rad([-360 360]));
            L3 = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-180 180]));
            L4 = Link('d',0.515,'a',0,'alpha',pi/2,'qlim',deg2rad([-200 200]));
            L5 = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-125 125]));
            L6 = Link('d',0.11,'a',0,'alpha',0,'qlim',deg2rad([-360 360]));
            
            pause(0.0001)
            self.model = SerialLink([L1 L2 L3 L4 L5 L6], 'name', name);
        end
        
        function [t] = withinBounds(self, q)
            %             self.currentJoints = self.model.getpos();
            Joints = q;
            t = 1;
            
            %set current cords to be base location
            current_link = self.location;
            
            %iterate through all link pose to find if any go under the
            %table
            for joint = 1:self.model.n
                current_link = current_link * self.model.A(joint,Joints);
                if(current_link(3,4) < 0)
                    %if true return 0
                    t = 0;
                    return
                end
            end
        end
        
        %% Joystick functionality (Per Robot Arm)
        function setJoy(self, joyObj, joy, NOJOY)
            self.joyObj = joyObj;
            self.joy = joy;
            self.NOJOY = NOJOY;
        end
        
        function [value] = checkJoy(self)
            [self.axes, self.buttons, self.povs] = self.joyObj.JoystickRead(self.joy);
            % buttons (1,2) corresponds to RED B button on LOGITECH
            % controller
            if self.buttons(1,2) == 0
                value = 0;
            else
                value = 1;
            end
        end
        
        function [newEndEffector] = joggingLoop(self, increments, debug)
            % Each time this jogging function is run, it will obtain the
            % end effector position, and apply a slight offset to it.
            % 
            % Run this in a while loop
            %   Check stop > Run joggingLoop > Apply transform
            [self.axes, self.buttons, self.povs] = self.joyObj.JoystickRead(self.joy);
            currentEndPose = self.getPose();
            
            % For each button state, apply a certain offset to the current
            % end effector transform, and calculate the joint positions
            % from that
            
            newEndEffector = currentEndPose;
            
            if (self.axes(1) > 0.5)
                newEndEffector = currentEndPose * transl([increments, 0, 0]);
                if debug == 1
                    disp(newEndEffector)
                end
            end
            
            if (self.axes(1) < -0.5)
                newEndEffector = currentEndPose * transl([-increments, 0, 0]);
                if debug == 1
                    disp(newEndEffector)
                end
            end
            
            if (self.axes(2) > 0.5)
                newEndEffector = currentEndPose * transl([0, increments, 0]);
                if debug == 1
                    disp(newEndEffector)
                end
            end
            
            if (self.axes(2) < -0.5)
                newEndEffector = currentEndPose * transl([0, -increments, 0]);
                if debug == 1
                    disp(newEndEffector)
                end
            end
            
            if (self.buttons(4) == 1)
                newEndEffector = currentEndPose * transl([0, 0, increments]);
                if debug == 1
                    disp(newEndEffector)
                end
            end
            
            if (self.buttons(1) == 1)
                newEndEffector = currentEndPose * transl([0, 0, -increments]);
                if debug == 1
                    disp(newEndEffector)
                end
            end
            
            q  = self.model.ikcon(newEndEffector, self.model.getpos);
            self.model.animate(q);
        end
    end
end