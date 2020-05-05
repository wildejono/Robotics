function Ch2Ex1()
close all;
clc

%% Transform 1
T1 = se2(1, 2, 30*pi/180)
T1_h = trplot2(T1, 'frame', '1', 'color', 'b');

%% Transform 2
T2 = se2(2, 1, 0)
hold on
T2_h = trplot2(T2, 'frame', '2', 'color', 'r');

%% Transform 3 is T1*T2
T3 = T1*T2
T3_h =trplot2(T3, 'frame', '3', 'color', 'g');

%% Axis
axis([0 5 0 5]);
axis equal;
grid on;

%% Move Transform 2
for i=90:-0.5:0
    T1 = se2(1, 2, i*pi/180)
    delete(T1_h);
    T1_h = trplot2(T1, 'frame', '1', 'color', 'b');
    
%     T2 = se2(i, 1, 0)
%     delete(T2_h);
%     T2_h = trplot2(T2, 'frame', '2', 'color', 'r');

    T3 = T1*T2
    delete(T3_h);
    T3_h =trplot2(T3, 'frame', '3', 'color', 'g');
    drawnow();
    pause(0.01);
end