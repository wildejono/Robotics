function tutorial01()
%% Clear stage
close all;
clc

%% Establish frame
axis([0 5 0 5]);
axis equal;
grid on;
pause

%% Set up transform 1
T1 = se2(0, 0, 0)
T1_h = trplot2(T1, 'frame', '1', 'color', 'b');
pause
end