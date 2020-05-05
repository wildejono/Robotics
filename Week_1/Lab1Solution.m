function Lab1Solution(  )
close all;
%% Question 1
% Download and setup the toolbox. See videos and links from UTSOnline

%% Question 2
set(gcf,'Name',['Question ',num2str(2)])
imshow('W1LabEx1_CircularRaceTrack.jpg');
axis on
hold on;

car1Tr = se2(300, 550, 0);
car1Tr_h = trplot2(car1Tr, 'frame', '1', 'color', 'b','length',50);

% The track diameter to outside lane is (550-66) = 484
% Approx circumference = pi * 484 = 1521
forLoopIncrements = 360;

% So the transform each step is
car1MoveTr = se2((pi * 484)/forLoopIncrements, 0, 0);
car1TurnTr = se2(0, 0, -2*pi/forLoopIncrements);

for i = 1:forLoopIncrements
    car1Tr = car1Tr * car1MoveTr * car1TurnTr;
    try delete(car1Tr_h);end
    try delete(text_h);end
    car1Tr_h = trplot2(car1Tr, 'frame', '1', 'color', 'b','length',50);
    message = sprintf([num2str(round(car1Tr(1,:),2,'significant')),'\n' ...
                      ,num2str(round(car1Tr(2,:),2,'significant')),'\n' ...
                      ,num2str(round(car1Tr(3,:),2,'significant'))]);
    text_h = text(10, 50, message, 'FontSize', 10, 'Color', [.6 .2 .6]);
    drawnow();
end

%% 3 and 4
for question = 3:4
    set(gcf,'Name',['Question ',num2str(question)])
    if question == 4
        subplot(1,2,1);
    end
    imshow('W1LabEx1_CircularRaceTrack.jpg');
    axis on
    hold on;
    car1Tr = se2(300, 550, 0);
    car2Tr = se2(300, 125, 0);
    
    if question == 4        
        % For distance plot (question 4)
        subplot(1,2,2);
        xlabel('Timestep');
        ylabel('Sensor reading - distance between cars');
        hold on;        
    end     

    % The track diameter to outside lane is (550-66) = 484
    % Approx circumference = pi * 484 = 1521
    % The track diameter to inside lane is (500 - 125) = 375
    % Approx circumference = pi * 375 = 1178

    forLoopIncrements = 360;
    % So the transform each step is
    car1MoveTr = se2((pi * 484)/forLoopIncrements, 0, 0);
    car1TurnTr = se2(0, 0, -2*pi/forLoopIncrements);
    car2MoveTr = se2((pi * 375)/forLoopIncrements, 0, 0);
    car2TurnTr = se2(0, 0, 2*pi/forLoopIncrements);
    dist = zeros(1,forLoopIncrements);

    for i = 1:forLoopIncrements
        car1Tr = car1Tr * car1MoveTr * car1TurnTr;
        car2Tr = car2Tr * car2MoveTr * car2TurnTr;    
        
        car1_to_2Tr = inv(car1Tr) * car2Tr
        car2_to_1Tr = inv(car2Tr) * car1Tr
        
        if question == 4
            subplot(1,2,1);
        end
        try delete(car1Tr_h);end
        try delete(car2Tr_h);end  
        car1Tr_h = trplot2(car1Tr, 'frame', '1', 'color', 'b','length',50);
        car2Tr_h = trplot2(car2Tr, 'frame', '2', 'color', 'r','length',50);
        if question == 4            
            subplot(1,2,2);
            try delete(distPlot_h);end
            dist(i) = sqrt(sum((car1Tr(1:2,3) - car2Tr(1:2,3)).^2));
            distPlot_h = plot(1:i,dist(1:i),'b-');
        end
        
        drawnow();
    end
end

