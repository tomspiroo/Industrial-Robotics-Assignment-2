clear all 
close all

set(0,'DefaultFigureWindowStyle','docked')
gui = GUI_App;
ur3 = UR3;
q1 = [0 0 0 0 0 0];

hold on
ur3.model.base = transl(0.25,0.4,0);
ur3.model.plot(q1);

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
QMatrix = jtraj(q1, q2, 50);
for i = 1:50
    if gui.EditFieldMotion.Value == "Robot in motion"
        ur3.model.animate(QMatrix(i,:));
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robot stopped"
            ur3.model.plot(QMatrix(i,:));
        end
    end
end
disp('1. Under first liquid')
q1 = q2;

% Collect first liquid
Tr2 = Tr * transl(0,0.1,0);
q2 = ur3.model.ikcon(Tr2);
QMatrix = jtraj(q1, q2, 50);
for i = 1:50
    if gui.EditFieldMotion.Value == "Robot in motion"
        ur3.model.animate(QMatrix(i,:));
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robot stopped"
            ur3.model.plot(QMatrix(i,:));
        end
    end
end
disp('2. Collected first liquid')


QMatrix = jtraj(q2, q1, 50);
for i = 1:50
    if gui.EditFieldMotion.Value == "Robot in motion"
        ur3.model.animate(QMatrix(i,:));
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robot stopped"
            ur3.model.plot(QMatrix(i,:));
        end
    end
end
disp('3. Moved back down')

% Move to location under second liquid dispenser
Tr3 = Tr * transl(0.15,0,0);
q2 = ur3.model.ikcon(Tr3);
QMatrix = jtraj(q1, q2, 50);
for i = 1:50
    if gui.EditFieldMotion.Value == "Robot in motion"
        ur3.model.animate(QMatrix(i,:));
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robot stopped"
            ur3.model.plot(QMatrix(i,:));
        end
    end
end
disp('4. Under second liquid')
q1 = q2;

% Collect second liquid 
Tr4 = Tr3 * transl(0,0.1,0);
q2 = ur3.model.ikcon(Tr4);
QMatrix = jtraj(q1, q2, 50);
for i = 1:50
    if gui.EditFieldMotion.Value == "Robot in motion"
        ur3.model.animate(QMatrix(i,:));
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robot stopped"
            ur3.model.plot(QMatrix(i,:));
        end
    end
end
disp('5. Collected second liquid')
QMatrix = jtraj(q2, q1, 50);
for i = 1:50
    if gui.EditFieldMotion.Value == "Robot in motion"
        ur3.model.animate(QMatrix(i,:));
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robot stopped"
            ur3.model.plot(QMatrix(i,:));
        end
    end
end
disp('6. Moved back down')

% Return to origin
q2 = [0 0 0 0 0 0];
QMatrix = jtraj(q1, q2, 50);
for i = 1:50
    if gui.EditFieldMotion.Value == "Robot in motion"
        ur3.model.animate(QMatrix(i,:));
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robot stopped"
            ur3.model.plot(QMatrix(i,:));
        end
    end
end
disp('7. Returned to origin')

% function estop (robot, value, q)
%     while value == "Robot stopped"
% %         eff = ur3.model.fkine(q);
% %         q2 = ur3.model.ikcon(eff);
%         ur3.model.plot(q);
%     end
% 
% end