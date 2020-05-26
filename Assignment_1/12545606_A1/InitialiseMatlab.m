function InitialiseMatlab()
%INITIALISEWORKSPACE Initialise workspace
%   Dock figures, clear all and
clear all;
clc;
clf;
set(0,'DefaultFigureWindowStyle','docked');
% FOR MAC
run /Users/jonathanwilde/git/Robotics/rvctools/startup_rvc.m
% FOR WINDOWS
%run 'C:\Users\Jay\MATLAB Drive\RVC1\rvctools\startup_rvc.m'
end

