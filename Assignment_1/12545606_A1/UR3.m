classdef UR3 < handle
    properties
        %> Robot model
        model;
        
        %> workspace
        workspace = [-5 5 -4 4 -0.78 2];
               
        %> If we have a tool model which will replace the final links model, combined ply file of the tool model and the final link models
        toolModelFilename = []; % Available are: 'DabPrintNozzleTool.ply';        
        toolParametersFilename = []; % Available are: 'DabPrintNozzleToolParameters.mat';        
    end 
    
    methods%% Class for UR3 robot simulation
        function self = UR3(base)
%             if 0 < nargin
%                 if length(toolModelAndTCPFilenames) ~= 2
%                     error('Please pass a cell with two strings, toolModelFilename and toolCenterPointFilename');
%                 end
%                 self.toolModelFilename = toolModelAndTCPFilenames{1};
%                 self.toolParametersFilename = toolModelAndTCPFilenames{2};
%             end
            
            self.GetUR3Robot(base);
            self.PlotAndColourRobot();%robot,workspace);

            drawnow
            % camzoom(2)
            % campos([6.9744    3.5061    1.8165]);

%             camzoom(4)
%             view([122,14]);
%             camzoom(8)
%             teach(self.model);
        end

        %% GetUR3Robot
        % Given a name (optional), create and return a UR3 robot model
        function GetUR3Robot(self,base)
            pause(0.001);
            name = ['UR_3_',datestr(now,'yyyymmddTHHMMSSFFF')];

            L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',[-2*pi 2*pi],'offset',0);
            L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim',[-2*pi 2*pi],'offset',0);
            L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim',[-2*pi 2*pi],'offset',0);
            L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',[-2*pi 2*pi],'offset',0);
            L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',[-2*pi 2*pi],'offset',0);
            L6 = Link('d',0.0819,'a',0,'alpha',0,'qlim',[-2*pi 2*pi],'offset',0);

            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',name,'base',base);
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['models/r3/L',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            if ~isempty(self.toolModelFilename)
                [ faceData, vertexData, plyData{self.model.n + 1} ] = plyread(self.toolModelFilename,'tri'); 
                self.model.faces{self.model.n + 1} = faceData;
                self.model.points{self.model.n + 1} = vertexData;
                toolParameters = load(self.toolParametersFilename);
                self.model.tool = toolParameters.tool;
                self.model.qlim = toolParameters.qlim;
                warning('Please check the joint limits. They may be unsafe')
            end
            % Display robot
%             self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            self.model.plot3d([-pi/2 -pi/2 pi/2 -pi/2 -pi/2 0],'noarrow','workspace',self.workspace,'scale',0);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
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
        
        function MoveHome(self)
            steps = 50;
            % Define End-Effector transformation, use inverse kinematics to get joint angles
            q1 = self.model.getpos();
            q2 = [-pi/2 -pi/2 pi/2 -pi/2 -pi/2 0];
            
            % Interpolate joint angles, also calculate relative velocity, accleration
            qMatrix = jtraj(q1,q2,steps);
            
            hold on
            for i = 1:length(qMatrix)
                self.model.plot(qMatrix(i,:),'noarrow','scale',0)
                %disp(qMatrix(i,:))
            end
            disp('The target position has been achieved!')
            disp(self.model.fkine(qMatrix(length(qMatrix),:)))
            hold off
        end
        
        function [returnJointState] = DetermineJointState(self,passT)
            returnJointState = self.model.ikcon(passT)
        end
        
        function MoveRobot(self,passT2)
            steps = 50;
            % Define End-Effector transformation, use inverse kinematics to get joint angles
            q1 = self.model.getpos();
            q2 = self.model.ikcon(passT2,q1);
            
            % Interpolate joint angles, also calculate relative velocity, accleration
            qMatrix = jtraj(q1,q2,steps);
            
            hold on
            for i = 1:length(qMatrix)
                self.model.plot(qMatrix(i,:),'noarrow','scale',0)
                %disp(qMatrix(i,:))
            end
            disp('The target position has been achieved!')
            disp(self.model.fkine(qMatrix(length(qMatrix),:)))
            hold off
        end
        
        function MoveRobotGrab(self,passT2,mesh,vert,vertCount,mode)
            steps = 50;
            % Define End-Effector transformation, use inverse kinematics to get joint angles
            q1 = self.model.getpos();
            q2 = self.model.ikcon(passT2,q1);
            
            % Interpolate joint angles, also calculate relative velocity, accleration
            qMatrix = jtraj(q1,q2,steps);
            
            hold on
            for i = 1:length(qMatrix)
                self.model.plot(qMatrix(i,:),'noarrow','scale',0)
                %disp(qMatrix(i,:))
                
                % Calculate the position of the part to follow
                endEffector = self.model.fkine(qMatrix(i,:));
                
                if mode == 0
                    updatedPoints = [endEffector * [vert,ones(vertCount,1)]']';  
                else
                    randRotateTR = makehgtform('yrotate',pi);
                    updatedPoints = [(endEffector * randRotateTR) * [vert,ones(vertCount,1)]']';  
                end

                % Now update the Vertices
                mesh.Vertices = updatedPoints(:,1:3);
            end
            disp('The target position has been achieved!')
            disp(self.model.fkine(qMatrix(length(qMatrix),:)))
            hold off
        end
        
        function MoveRobotGrabAll(self,passT2,mesh1,mesh2,mesh3,...
                vert1,vert2,vert3,vertCount1,vertCount2,vertCount3,...
                mode1,mode2,mode3)
            steps = 50;
            % Define End-Effector transformation, use inverse kinematics to get joint angles
            q1 = self.model.getpos();
            q2 = self.model.ikcon(passT2,q1);
            
            % Interpolate joint angles, also calculate relative velocity, accleration
            qMatrix = jtraj(q1,q2,steps);
            
            hold on
            for i = 1:length(qMatrix)
                self.model.plot(qMatrix(i,:),'noarrow','scale',0)
                %disp(qMatrix(i,:))
                
                % Calculate the position of the part to follow
                endEffector = self.model.fkine(qMatrix(i,:));
                
                if mode1 == 0
                    forwardTR = makehgtform('translate',[0,0,0.02]);
                    updatedPoints1 = [(endEffector * forwardTR) * [vert1,ones(vertCount1,1)]']';  
                else
                    forwardTR = makehgtform('translate',[0,0,0.02]);
                    randRotateTR = makehgtform('yrotate',pi);
                    updatedPoints1 = [(endEffector * forwardTR * randRotateTR) * [vert,ones(vertCount,1)]']';  
                end
                
                mesh1.Vertices = updatedPoints1(:,1:3);
                
                if mode2 == 0
                    forwardTR = makehgtform('translate',[0,0,0]);
                    updatedPoints2 = [(endEffector * forwardTR) * [vert2,ones(vertCount2,1)]']';  
                else
                    forwardTR = makehgtform('translate',[0,0,0]);
                    randRotateTR = makehgtform('yrotate',pi);
                    updatedPoints2 = [(endEffector * forwardTR * randRotateTR) * [vert2,ones(vertCount2,1)]']';  
                end
                
                mesh2.Vertices = updatedPoints2(:,1:3);
                
                if mode3 == 0
                    forwardTR = makehgtform('translate',[0,0,0.04]);
                    updatedPoints3 = [(endEffector * forwardTR) * [vert3,ones(vertCount3,1)]']';  
                else
                    forwardTR = makehgtform('translate',[0,0,0.04]);
                    randRotateTR = makehgtform('yrotate',pi);
                    updatedPoints3 = [(endEffector * forwardTR * randRotateTR) * [vert3,ones(vertCount3,1)]']';
                end
                
                mesh3.Vertices = updatedPoints3(:,1:3);
            end
            disp('The target position has been achieved!')
            disp(self.model.fkine(qMatrix(length(qMatrix),:)))
            hold off
        end
    end
end