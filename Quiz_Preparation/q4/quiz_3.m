%% 
P=[2,2,2,2; -0.4,0.4,0.4,-0.4; 1.4,1.4,0.6,0.6]; 

pStar = [700 300 300 700; 300 300 700 700]; 

cam = CentralCamera('focal',0.08,'pixel',10e-5,'resolution',[1024 1024], 'centre',[512 512],'name','UR10camera');
r = UR10();

q0 =  [1.6; -1; -1.2; -0.5; 0; 0];
Tc0 = r.model.fkine(q0);
cam.T = Tc0;
uv = cam.plot(P);
e = pStar-uv


%% 
[X,Y] = meshgrid(-10:1:10,-10:1:10);
Z = X;
[x,y,z] = ellipsoid(3,2,1,5,5,5);
surf(X,Y,Z)
hold on
surf(x,y,z)
%% clc 
mdl_puma560
q = [0 2.3562 -3.0159 0 -0.9076 0]
	%q = [0 1.5708 -3.0159 0.1466 0.5585 0]
    
		%q = [1.1170 1.0996 -3.4872 0.1466 0.5585 0.6500]
        %q = [0 0.7 3 0 0.7 0]
jacobian = p560.jacob0(q);
measureOfManip = sqrt(det(jacobian(1:2,:)*jacobian(1:2,:)'))


%%
clear 
clc 

mdl_planar3 

stepCount = 50;
q1 = [-pi/3,0,0];
q2 = [pi/3,0,0];

trej = jtraj(q1,q2,stepCount)

[v,f,fn] = RectangularPrism([2,-1,-1], [3,1,1])

for i = 1:numel(trej)
    
    q = trej(i,:)
    
    result = IsCollision(p3,q,f,v,fn);
    
    if(result == 1)
        r = trej(i,:)
        break 
    end
end