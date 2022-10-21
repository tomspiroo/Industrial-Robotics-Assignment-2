clear all
close all

%% Workspace initiliasation
workspace = [-1 1 -0.1 1 -1 2];
BenchtopAndWall = GeneralModel('BenchtopAndWall','BenchtopAndWallPly.ply', transl(0,0,0), workspace);
Dispenser = GeneralModel('Dispenser','DispenserPly.ply', transl(0.45,0,0.42), workspace);
Bowl = GeneralModel('Bowl','BowlPly.ply', transl(-0.6,0.4,0), workspace);
GlassEmpty = GeneralModel('GlassEmpty','EmptyGlassPly.ply', transl(0.25,0.7,0), workspace);
GlassFull = GeneralModel('GlassFull','FullglassPly.ply', transl(-2,0.7,0), workspace);
Lime = GeneralModel('Lime','LimeSlicePly.ply', transl(-0.6,0.4,0.02), workspace);
drawnow

hold on
%% Robots and GUI initilisation
braccio = Braccio;
ur3 = UR3;
gui = GUI_App;

qbraccio = [0 0 0 0 0];
qur3 = [0 0 0 0 0 0];

braccio.model.base = transl(-0.25,0.4,0);
braccio.model.plot(qbraccio);
ur3.model.base = transl(0.5,0.4,0);
ur3.model.plot(qur3);


%% UR3 trajectory for collecting the cup
q1 = deg2rad([0 -5 0 0 0 0]);
q2 = deg2rad([-45 -4 0 0 -150 0]);

QMatrix = jtraj(q1,q2,50);
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
disp('1.1 Moved to location of cup');
QMatrix = jtraj(q2,q1,50);
for i = 1:50
        if gui.EditFieldMotion.Value == "Robot in motion"
            ur3.model.animate(QMatrix(i,:));
            GlassEmpty.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassEmpty.model.animate(0);
            drawnow()
        else
            while gui.EditFieldMotion.Value == "Robot stopped"
                ur3.model.plot(QMatrix(i,:));
                GlassEmpty.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)*  trotx(deg2rad(-90));
                GlassEmpty.model.animate(0);
            end
        end
end
disp('1.2 Cup collected, returned to origin')
%% UR3 trajectory for collecting liquids
% Move to location under first liquid dispenser 
Tr = ur3.model.base *  transl(-0.05,-0.3,0.4) * trotx(deg2rad(90));
q2 = ur3.model.ikcon(Tr);
QMatrix = jtraj(q1, q2, 50);
for i = 1:50
    if gui.EditFieldMotion.Value == "Robot in motion"
        ur3.model.animate(QMatrix(i,:));
            GlassEmpty.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassEmpty.model.animate(0);
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robot stopped"
            ur3.model.plot(QMatrix(i,:));
            GlassEmpty.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassEmpty.model.animate(0);
        end
    end
end
disp('2.1 Cup is under first liquid')
q1 = q2;

% Collect first liquid
Tr2 = Tr * transl(0,0.05,0);
q2 = ur3.model.ikcon(Tr2);
QMatrix = jtraj(q1, q2, 50);
for i = 1:50
    if gui.EditFieldMotion.Value == "Robot in motion"
        ur3.model.animate(QMatrix(i,:));
            GlassEmpty.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassEmpty.model.animate(0);
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robot stopped"
            ur3.model.plot(QMatrix(i,:));
            GlassEmpty.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassEmpty.model.animate(0);
        end
    end
end
disp('2.2 Collected first liquid')
GlassEmpty.model.base = [1 0 0 -2; 0 1 0 0; 0 0 1 0; 0 0 0 1];
GlassEmpty.model.animate(0);
QMatrix = jtraj(q2, q1, 50);
for i = 1:50
    if gui.EditFieldMotion.Value == "Robot in motion"
        ur3.model.animate(QMatrix(i,:));
            GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassFull.model.animate(0);
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robot stopped"
            ur3.model.plot(QMatrix(i,:));
            GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassFull.model.animate(0);
        end
    end
end
disp('2.3 Moved back down')

% Move to location under second liquid dispenser
Tr3 = Tr * transl(0.15,0,0);
q2 = ur3.model.ikcon(Tr3);
QMatrix = jtraj(q1, q2, 50);
for i = 1:50
    if gui.EditFieldMotion.Value == "Robot in motion"
        ur3.model.animate(QMatrix(i,:));
        GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
        GlassFull.model.animate(0);
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robot stopped"
            ur3.model.plot(QMatrix(i,:));
            GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassFull.model.animate(0);
        end
    end
end
disp('2.4 Cup is under second liquid')
q1 = q2;

% Collect second liquid 
Tr4 = Tr3 * transl(0,0.05,0);
q2 = ur3.model.ikcon(Tr4);
QMatrix = jtraj(q1, q2, 50);
for i = 1:50
    if gui.EditFieldMotion.Value == "Robot in motion"
        ur3.model.animate(QMatrix(i,:));
        GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
        GlassFull.model.animate(0);
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robot stopped"
            ur3.model.plot(QMatrix(i,:));
            GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassFull.model.animate(0);
        end
    end
end
disp('2.5 Collected second liquid')
QMatrix = jtraj(q2, q1, 50);
for i = 1:50
    if gui.EditFieldMotion.Value == "Robot in motion"
        ur3.model.animate(QMatrix(i,:));
        GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
        GlassFull.model.animate(0);
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robot stopped"
            ur3.model.plot(QMatrix(i,:));
            GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassFull.model.animate(0);
        end
    end
end
disp('2.6 Moved back down')

% Return to origin
q2 = deg2rad([0 -5 0 0 0 0]);
QMatrix = jtraj(q1, q2, 50);
for i = 1:50
    if gui.EditFieldMotion.Value == "Robot in motion"
        ur3.model.animate(QMatrix(i,:));
        GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
        GlassFull.model.animate(0);
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robot stopped"
            ur3.model.plot(QMatrix(i,:));
            GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassFull.model.animate(0);
        end
    end
end
disp('2.7 Returned to origin')

%% UR3 trajectory for placing filled cup in payload area
q1 = deg2rad([0 -5 0 0 0 0]);
q2 = deg2rad([-45 -4 0 0 -150 0]);

QMatrix = jtraj(q1,q2,50);
for i = 1:50
        if gui.EditFieldMotion.Value == "Robot in motion"
            ur3.model.animate(QMatrix(i,:));
            GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassFull.model.animate(0);
            drawnow()
        else
            while gui.EditFieldMotion.Value == "Robot stopped"
                ur3.model.plot(QMatrix(i,:));
                GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
                GlassFull.model.animate(0);
            end
        end
end
disp('3.1 Moved to payload area and dropped the payload');
QMatrix = jtraj(q2,q1,50);
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
disp('3.2 UR3 moves mack to origin')
