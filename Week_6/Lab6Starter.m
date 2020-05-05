%% Lab 6 Starter
classdef Lab6Starter < handle
    properties (Constant)
        % Normal of the plane       
        pNormal = [0, 1, 0]; % Make sure one one value is 1 (or -1) others are zero
    end
    
    properties
        % Point on the plane       
        pPoint = [0,1.55,0]; 
            
        %> Robot handle
        robot;
        
        %> Line plot handles
        normalLine_h;
        orientLine_h;
        approachLine_h;
        
        %> Point plot handles
        normalPoint_h;
        orientPoint_h;
        approachPoint_h;    
        
        %> Handles for the intersection point
        intersectP_h
        
        %> Robot start pose
        defaultQ = [0,pi/2,0,0,0,0];
        
        %> This is the length of the ray from the end effector
        rayLength = 10;
        
        %> Length of the lines used for the legs of the tr plot 
        trLineLength = 0.1;
    end

    methods
        function self = Lab6Starter()
            cla

            % Create the robot
            self.robot = SchunkUTSv2_0();            
            self.robot.plot3d(self.defaultQ);
            self.robot.delay = 0;
            hold on;
            
            % Plot plane at pPoint with normal along axis where pNormal is 0
            ellipsoidLengths = (self.pNormal==0);
            ellipsoid(self.pPoint(1),self.pPoint(2),self.pPoint(3) ...
                    ,ellipsoidLengths(1),ellipsoidLengths(2),ellipsoidLengths(3));
            
            view(3);
            camlight;
        end
        
        %% Test the class
        function Test1(self)            
            self.Reset();
            q = self.robot.getpos();
            % Move joint 1, 1 degree at a time and plot intersections
            for q1 = -pi/8:pi/180:pi/8
                self.robot.animate([q1,q(2:end)]);
                self.PlotApproachVectorIntersection();
                self.PlotTr();
                drawnow();
            end
        end
        
        %% Test the class
        function Test2(self)
            self.Reset();                       
            q = self.robot.getpos();
            
            % Move joint 1 and 5 to sweep and tilt, 5 degrees at a time and plot intersections           
            stepSize = 5 * pi/180;
            for q1 = -pi/8:stepSize:pi/8
                for q5 = -pi/8:stepSize:pi/8
                    q(1) = q1; 
                    q(5) = q5; 
                    self.robot.animate(q);
                    self.PlotApproachVectorIntersection();
                    self.PlotTr();
                    drawnow();
                end
            end
        end        
        
        %% Reset
        function Reset(self)
            self.DeleteIntersectPlots();
            self.DeleteTrPlots();
            self.robot.animate(self.defaultQ);
            drawnow();
        end
            
        %% PlotApproachVectorIntersection
        function PlotApproachVectorIntersection(self)
            tr = self.robot.fkine( self.robot.getpos );
            startP = tr(1:3,4)';
            endP = tr(1:3,4)' + self.rayLength * tr(1:3,3)';                             
            intersectP = LinePlaneIntersection(self.pNormal,self.pPoint,startP,endP);  
            self.intersectP_h(end+1) = plot3(intersectP(1),intersectP(2),intersectP(3),'c*');
            
            % Change the title
            dist=sqrt((startP(1)-intersectP(1)).^2+...
                      (startP(2)-intersectP(2)).^2+...
                      (startP(3)-intersectP(3)).^2);
            title(['Distance to intersection point is ',num2str(dist)]);            
        end
        
        %% Delete the intersection plots
        function DeleteIntersectPlots(self)
            % Assuming all have the same dimensions go through and delete them
            for i = 1:length(self.intersectP_h)
                try delete(self.intersectP_h(i));end %#ok<*TRYNC>
            end
        end
        
        %% Plot the tr axes for the current pose
        function PlotTr(self)
            tr = self.robot.fkine( self.robot.getpos );
            
            % The axes for the coordinate frame on the end effector are called 
            % "normal", "orient" and "approach" for the local X,Y,Z axes of the coordinate frame            
            startP = tr(1:3,4)';
            normalEndP = startP + self.trLineLength * tr(1:3,1)';
            orientEndP = startP + self.trLineLength * tr(1:3,2)';
            approachEndP = startP + self.trLineLength * tr(1:3,3)';
            
            % Plot normal orient and approach with red, green and blue
            self.normalLine_h(end+1) = plot3([startP(1),normalEndP(1)],[startP(2),normalEndP(2)],[startP(3),normalEndP(3)],'r');
            self.orientLine_h(end+1) = plot3([startP(1),orientEndP(1)],[startP(2),orientEndP(2)],[startP(3),orientEndP(3)],'g');
            self.approachLine_h(end+1) = plot3([startP(1),approachEndP(1)],[startP(2),approachEndP(2)],[startP(3),approachEndP(3)],'b');

            % Plot the point
            self.normalPoint_h(end+1) = plot3(normalEndP(1),normalEndP(2),normalEndP(3),'r*');
            self.orientPoint_h(end+1) = plot3(orientEndP(1),orientEndP(2),orientEndP(3),'g*');
            self.approachPoint_h(end+1) = plot3(approachEndP(1),approachEndP(2),approachEndP(3),'b*');
        end

        %% Delete the tr plots
        function DeleteTrPlots(self)
            % Assuming all have the same dimensions go through and delete them
            for i = 1:length(self.normalLine_h)
                try delete(self.normalLine_h(i));end %#ok<*TRYNC>
                try delete(self.orientLine_h(i));end
                try delete(self.approachLine_h(i));end
                try delete(self.normalPoint_h(i));end
                try delete(self.orientPoint_h(i));end
                try delete(self.approachPoint_h(i));end
            end
        end
    end
end