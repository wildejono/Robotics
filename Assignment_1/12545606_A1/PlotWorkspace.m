% function [pcbMat,topMat,botMat,pcbVerts,pcbVertCount,topHousingVerts,topHousingVertCount,...
%     botHousingVerts,botHousingVertCount] = PlotWorkspace(pcbOffset, topOffset, botOffset)
function [pcbVerts,pcbVertCount,topHousingVerts,topHousingVertCount,...
    botHousingVerts,botHousingVertCount,pcbMesh,topMesh,botMesh] = PlotWorkspace(pcbOffset, topOffset, botOffset, tableOffset)
%PLOTWORKSPACE Summary of this function goes here
%   Detailed explanation goes here

%% Load images as background and surface of workspace
floorIm = imread('images/concrete.jpg');
wallIm1 = imread('images/factory.jpg');
% Publish the walls
xImage1 = [-5, 5];
yImage1 = [-4, -4];
zImage1 = [2, 2; -0.76, -0.76];
surf(xImage1, yImage1, zImage1,...
    'CData',wallIm1,...
    'FaceColor','texturemap');

% Publish the floor
xImage = [-5, 5];
yImage = [-4, 4];
zImage = [-0.76, -0.76; -0.76, -0.76];
hold on
surf(xImage, yImage, zImage,...
    'CData',floorIm,...
    'FaceColor','texturemap');
hold off

%% Master table offset
coneToTableOffset = [-3,-2,0;0,-2,0;3,-2,0;...
    -3,2,0;0,2,0;3,2,0];

%% Import table
[f,v,data] = plyread('models/table.ply','tri');
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
% Then plot the trisurf
hold on
tableMeshH = trisurf(f,v(:,1) + tableOffset(1),v(:,2) + tableOffset(2),v(:,3) + tableOffset(3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold off

%% Import cones
for i = 1:6
    [f,v,data] = plyread('models/cone.ply','tri');
    % Scale the colours to be 0-to-1 (they are originally 0-to-255
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    % Then plot the trisurf
    hold on
    coneH = trisurf(f,v(:,1) + tableOffset(1) + coneToTableOffset(i,1),v(:,2) + ...
        tableOffset(2) + coneToTableOffset(i,2),v(:,3) + tableOffset(3) + coneToTableOffset(i,3) ...
        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
end

%% Import box
[f,v,data] = plyread('models/box.ply','tri');
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
% Then plot the trisurf
hold on
boxH = trisurf(f,v(:,1)+0.6,v(:,2)+0.2,v(:,3)+0.06 ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold off

%% Import three parts for assembly

[f,v,data] = plyread('models/PCB.ply','tri');
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
pcbVertCount = size(v,1);
midPoint = sum(v)/pcbVertCount;
pcbVerts = v - repmat(midPoint,pcbVertCount,1);
% Then plot the trisurf
hold on
pcbMesh = trisurf(f,pcbVerts(:,1) + pcbOffset(1),pcbVerts(:,2) + pcbOffset(2),pcbVerts(:,3) + pcbOffset(3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold off

[f,v,data] = plyread('models/TopHousing.ply','tri');
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
topHousingVertCount = size(v,1);
midPoint = sum(v)/topHousingVertCount;
topHousingVerts = v - repmat(midPoint,topHousingVertCount,1);
% Then plot the trisurf
hold on
topMesh = trisurf(f,topHousingVerts(:,1) + topOffset(1),topHousingVerts(:,2) + topOffset(2),topHousingVerts(:,3) + topOffset(3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold off

[f,v,data] = plyread('models/BotHousing.ply','tri');
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
botHousingVertCount = size(v,1);
midPoint = sum(v)/botHousingVertCount;
botHousingVerts = v - repmat(midPoint,botHousingVertCount,1);
% Then plot the trisurf
hold on
botMesh = trisurf(f,botHousingVerts(:,1) + botOffset(1),botHousingVerts(:,2) + botOffset(2),botHousingVerts(:,3) + botOffset(3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold off

end

