clear all 
close all

set(0,'DefaultFigureWindowStyle','docked')

ur3 = UR3;
origin = [0 0 0 0 0 0];

hold on
ur3.model.base = transl(0.25,0.4,0);
ur3.model.plot(origin);

% Create Wall Surface
wallPoint = [0,0,0];
wallNormal = [0,-1,0];
wallvertx = [1,-1;1,-1];
wallverty = [0,0;0,0];
wallvertz = [-1,-1;1,1];
wall_h = surf(wallvertx,wallverty,wallvertz);
hold off

%% Trajectory for collecting liquids

% Move to location under first liquid dispenser 
Tr = ur3.model.base *  transl(-0.05,-0.3,0.4) * trotx(deg2rad(90));
q2 = ur3.model.ikcon(Tr);
QMatrix1 = jtraj(origin, q2, 50);
for i = 1:50
        ur3.model.animate(QMatrix1(i,:));
        drawnow()
end
disp('1. Under first liquid')

% Collect first liquid
Tr2 = Tr * transl(0,0.1,0);
q3 = ur3.model.ikcon(Tr2);
QMatrix2 = jtraj(q2, q3, 50);
for i = 1:50
        ur3.model.animate(QMatrix2(i,:));
        drawnow()
end
disp('2. Collected first liquid')
QMatrix3 = jtraj(q3, q2, 50);
for i = 1:50
        ur3.model.animate(QMatrix3(i,:));
        drawnow()
end
disp('3. Moved back down')

% Move to location under second liquid dispenser
Tr3 = Tr * transl(0.15,0,0);
q4 = ur3.model.ikcon(Tr3);
QMatrix4 = jtraj(q2, q4, 50);
for i = 1:50
        ur3.model.animate(QMatrix4(i,:));
        drawnow()
end
disp('4. Under second liquid')

% Collect second liquid 
Tr4 = Tr3 * transl(0,0.1,0);
q5 = ur3.model.ikcon(Tr4);
QMatrix5 = jtraj(q4, q5, 50);
for i = 1:50
        ur3.model.animate(QMatrix5(i,:));
        drawnow()
end
disp('5. Collected second liquid')
QMatrix6 = jtraj(q5, q4, 50);
for i = 1:50
        ur3.model.animate(QMatrix6(i,:));
        drawnow()
end
disp('6. Moved back down')

% Return to origin
QMatrix7 = jtraj(q4, origin, 50);
for i = 1:50
        ur3.model.animate(QMatrix7(i,:));
        drawnow()
end
disp('7. Returned to origin')