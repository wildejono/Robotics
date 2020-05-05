function Lab4_Puma560()

mdl_puma560

T1 = transl(0.5,-0.4,0.5);

q1 = p560.ikine(T1);

T2 = transl(0.5,0.4,0.1);

q2 = p560.ikine(T2);

steps = 50;

% Method one
%qMatrix = jtraj(q1,q2,steps)

% Method two
s = lspb(0,1,steps); qMatrix = nan(steps,6); 
for i = 1:steps
    qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
end

figure(1)
p560.plot(qMatrix,'trail','r-')

velocity = zeros(steps,6); acceleration = zeros(steps,6);
for i = 2:steps
    velocity(i,:) = qMatrix(i,:) - qMatrix(i-1,:);
    acceleration(i,:) = velocity(i,:) - velocity(i-1,:);
end

for i = 1:6
   figure(2)
    subplot(3,2,i)
    plot(qMatrix(:,i),'k','LineWidth',1)
    title(['Joint ', num2str(i)])
    xlabel('Step')
    ylabel('Joint Angle (rad)')
    refline(0,p560.qlim(i,1)) 
    refline(0,p560.qlim(i,2))
    % Reference line on the lower joint limit for joint i
    % Reference line on the upper joint limit for joint i
    figure(3)
    subplot(3,2,i)
    plot(velocity(:,i),'k','LineWidth',1)
    title(['Joint ', num2str(i)])
    xlabel('Step')
    ylabel('Joint Velocity')
    figure(4)
    subplot(3,2,i)
    plot(acceleration(:,i),'k','LineWidth',1)
    title(['Joint ', num2str(i)])
    xlabel('Step')
    ylabel('Joint Acceleration') 
end