%% Puma 560 DH Model
% 41013 Robotics
% Jonathan Woolfrey
% August 2016

% Link('theta',__,'d',__,'a',__,'alpha',__,'offset',__,'qlim',[ ... ])

L1 = Link('d',0,'a',0,'alpha',pi/2,'offset',0)

L2 = Link('d',0,'a',0.4318,'alpha',0,'offset',0)

L3 = Link('d',0.15,'a',0.0203,'alpha',-pi/2,'offset',0)

L4 = Link('d',0.4318,'a',0,'alpha',pi/2,'offset',0)

L5 = Link('d',0,'a',0,'alpha',-pi/2,'offset',0)

L6 = Link('d',0,'a',0,'alpha',0,'offset',0)

myRobot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'Puma560')

q = zeros(1,6)

myRobot.plot(q)