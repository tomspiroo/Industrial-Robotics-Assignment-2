clear all
close all

%% Workspace initiliasation
workspace = [-1 1 -0.1 1 -1 2];
barrier1 = surf([-1,-1;0,0],[0.8,0.8;0.8,0.8],[1,0;1,0],'CData',imread('glass.jpg'),'FaceColor','texturemap'); %Creates tabletop surface
barrier1.FaceAlpha = 'texturemap';
barrier1.AlphaData = 1;
hold on
barrier2 = surf([0,0;0.45,0.45],[0.8,0.8;0.8,0.8],[1,0.25;1,0.25],'CData',imread('glass.jpg'),'FaceColor','texturemap'); %Creates tabletop surface
barrier2.FaceAlpha = 'texturemap';
barrier2.AlphaData = 1;
barrier3 = surf([0.45,0.45;1,1],[0.8,0.8;0.8,0.8],[1,0;1,0],'CData',imread('glass.jpg'),'FaceColor','texturemap'); %Creates tabletop surface
barrier3.FaceAlpha = 'texturemap';
barrier3.AlphaData = 1;
% Perspex collision check data
barrierVertex(1,:) = [-1,0.8,0];
barrierVertex(2,:) = [1,0.8,0];
barrierVertex(3,:) = [-1,0.8,1];
barrierVertex(4,:) = [1,0.8,1];
barrierFace = [1,2,3;1,3,4];
barrierFaceNormals = zeros(size(barrierFace,1),3);
for faceIndex = 1:size(barrierFace,1)
    v1 = barrierVertex(barrierFace(faceIndex,1)',:);
    v2 = barrierVertex(barrierFace(faceIndex,2)',:);
    v3 = barrierVertex(barrierFace(faceIndex,3)',:);
    barrierFaceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
end
%NOTE: Collision check for barrier would be IsCollision(ur3.model,qMatrix,barrierFace,barrierVertex,barrierFaceNormals,false)
BenchtopAndWall = GeneralModel('BenchtopAndWall','BenchtopAndWallPly.ply', transl(0,0,0), workspace);
Dispenser = GeneralModel('Dispenser','DispenserPly.ply', transl(0.45,0,0.42), workspace);
Bowl = GeneralModel('Bowl','BowlPly.ply', transl(-0.25,0.7,0), workspace);
GlassEmpty = GeneralModel('GlassEmpty','EmptyGlassPly.ply', transl(0.25,0.7,0), workspace);
GlassFull = GeneralModel('GlassFull','FullglassPly.ply', transl(0,0.25,-0.5), workspace);
Lime = GeneralModel('Lime','LimeSlicePly.ply', transl(-0.25,0.7,0.04), workspace);
Interrupt = GeneralModel('Interrupt','Interrupt.ply',transl(0.25,0.4,-0.5),workspace); %NEEDS TO BE LOCATED CORRECTLY AND MODEL CREATED
drawnow

hold on
%% Robots and GUI initilisation
braccio = Braccio;
ur3 = UR3;
gui = GUI_App;
gui2 = Drink_selection_gui;

qbraccio = deg2rad([0 15 0 0 0]);
qur3 = deg2rad([0 -5 0 0 0 0]);

braccio.model.base = transl(-0.25,0.4,0);
braccio.model.plot(qbraccio);
ur3.model.base = transl(0.5,0.4,0);
ur3.model.plot(qur3);


%% UR3 trajectory for collecting the cup
q1 = deg2rad([0 -5 0 0 0 0]);
q2 = deg2rad([-45 -4 0 0 -150 0]);

QMatrix = jtraj(q1,q2,25); %Calculate Trajectory
QMatrix2 = qbraccio;

%% Check for collisions
% collisionCheck = IsModelCollision(ur3,BenchtopAndWall,QMatrix);
% if collisionCheck == 1
%     gui.EditFieldMotion.Value = "Obstacle found";
%     display("Potential collision identified. Testing known safe waypoints for alternative.")
% end
%% Run movement
while gui2.BSel.Value ~= "Bitters" && gui2.BSel.Value ~= "No bitters" && gui2.SWSel.Value ~= "Soda water" && gui2.SWSel.Value ~= "No soda water"
    braccio.model.plot(qbraccio);
    ur3.model.plot(qur3);
end
% After start, enable light curtain
linex = [];
liney = [];
linez = [];
lineCoordIndex = 1;
for x = 0.05:0.05:0.40
    plot3([x x],[0.8 0.8],[0 0.25],'r');
    linex(lineCoordIndex) = x;
    liney(lineCoordIndex) = 0.8;
    linez(lineCoordIndex) = 0;
    lineCoordIndex = lineCoordIndex + 1;
    linex(lineCoordIndex) = x;
    liney(lineCoordIndex) = 0.8;
    linez(lineCoordIndex) = 0.25;
    lineCoordIndex = lineCoordIndex + 1;
end
obstructionCheck(ur3, QMatrix, braccio, QMatrix2, BenchtopAndWall, Interrupt, linex, liney, linez, barrierFace,barrierVertex,barrierFaceNormals);
for i = 1:25
        if gui.EditFieldMotion.Value == "Robots in motion"
            ur3.model.animate(QMatrix(i,:));
            drawnow()
        else
            while gui.EditFieldMotion.Value == "Robots stopped"
                ur3.model.plot(QMatrix(i,:));
            end
        end
end
disp('UR3: Moved to location of cup');
QMatrix = jtraj(q2,q1,25);
QMatrix2 = qbraccio;
obstructionCheck(ur3, QMatrix, braccio, QMatrix2, BenchtopAndWall, Interrupt, linex, liney, linez, barrierFace,barrierVertex,barrierFaceNormals);
for i = 1:25
        if gui.EditFieldMotion.Value == "Robots in motion"
            ur3.model.animate(QMatrix(i,:));
            GlassEmpty.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassEmpty.model.animate(0);
            drawnow()
        else
            while gui.EditFieldMotion.Value == "Robots stopped"
                ur3.model.plot(QMatrix(i,:));
                GlassEmpty.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)*  trotx(deg2rad(-90));
                GlassEmpty.model.animate(0);
            end
        end
end
disp('UR3: Cup collected, returned to origin')
%% UR3 trajectory for collecting liquids
% Move to location under first liquid dispenser 
Tr = ur3.model.base *  transl(-0.05,-0.3,0.4) * trotx(deg2rad(90));
q2 = ur3.model.ikcon(Tr);
QMatrix = jtraj(q1, q2, 50);
obstructionCheck(ur3, QMatrix, braccio, QMatrix2, BenchtopAndWall, Interrupt, linex, liney, linez, barrierFace,barrierVertex,barrierFaceNormals);
collisionCheck = IsModelCollision(ur3,BenchtopAndWall,QMatrix);
if collisionCheck == 1
    gui.EditFieldMotion.Value = "Obstacle found";
    disp("Potential collision identified. Testing known safe waypoints for alternative.")
    qWaypoint = deg2rad([0 -133 110 25 33 0]);
    QMatrixCheck1 = jtraj(q1, qWaypoint, 25);
    QMatrixCheck2 = jtraj(qWaypoint, q2, 25);
    collisionCheck1 = IsModelCollision(ur3,BenchtopAndWall,QMatrixCheck1);
    collisionCheck2 = IsModelCollision(ur3,BenchtopAndWall,QMatrixCheck2);
    if collisionCheck1 == 1 || collisionCheck2 == 1
        gui.EditFieldMotion.Value = "Failed pathing";
        disp("Failed to identify safe path. Robot halting.")
    else
        collisionCheck = 0;
        QMatrix = cat(1, QMatrixCheck1, QMatrixCheck2);
    end
    while ~gui.EditFieldMotion.Value == "Robots in motion"
        % Do nothing
    end
end
qWaypoint = deg2rad([0 -133 110 25 33 0]);
QMatrixCheck1 = jtraj(q1, qWaypoint, 25);
QMatrixCheck2 = jtraj(qWaypoint, q2, 25);
QMatrix = cat(1, QMatrixCheck1, QMatrixCheck2);
% Interrupt.model.base = transl(0.25,0.8,0.1);
% Interrupt.model.animate([0]);
% drawnow
% gui.EditFieldMotion.Value = "Failed pathing";
% disp("Failed to identify safe path. Robot halting.")
% while gui.EditFieldMotion.Value == "Failed pathing" 
%     %Do nothing
% end
%Interrupt.model.base = transl(0,0.4,-0.5);
%obstructionCheck(ur3, QMatrix, braccio, QMatrix2, BenchtopAndWall, Interrupt, linex, liney, linez, barrierFace,barrierVertex,barrierFaceNormals);
for i = 1:50
    if gui.EditFieldMotion.Value == "Robots in motion"
        ur3.model.animate(QMatrix(i,:));
            GlassEmpty.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassEmpty.model.animate(0);
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robots stopped"
            ur3.model.plot(QMatrix(i,:));
            GlassEmpty.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassEmpty.model.animate(0);
        end
    end
end
disp('UR3: Cup is under first liquid')
q1 = q2;
%% Continue
% Collect first liquid
Tr2 = Tr * transl(0,0.05,0);
q2 = ur3.model.ikcon(Tr2);
QMatrix = jtraj(q1, q2, 20);
obstructionCheck(ur3, QMatrix, braccio, QMatrix2, BenchtopAndWall, Interrupt, linex, liney, linez, barrierFace,barrierVertex,barrierFaceNormals);
for i = 1:20
    if gui.EditFieldMotion.Value == "Robots in motion"
        ur3.model.animate(QMatrix(i,:));
            GlassEmpty.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassEmpty.model.animate(0);
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robots stopped"
            ur3.model.plot(QMatrix(i,:));
            GlassEmpty.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassEmpty.model.animate(0);
        end
    end
end
disp('UR3: Collected first liquid')
GlassEmpty.model.base = [1 0 0 0; 0 1 0 0.25; 0 0 1 -0.5; 0 0 0 1];
GlassEmpty.model.animate(0);
QMatrix = jtraj(q2, q1, 20);
obstructionCheck(ur3, QMatrix, braccio, QMatrix2, BenchtopAndWall, Interrupt, linex, liney, linez, barrierFace,barrierVertex,barrierFaceNormals);
for i = 1:20
    if gui.EditFieldMotion.Value == "Robots in motion"
        ur3.model.animate(QMatrix(i,:));
            GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassFull.model.animate(0);
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robots stopped"
            ur3.model.plot(QMatrix(i,:));
            GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassFull.model.animate(0);
        end
    end
end
disp('UR3: Moved back down')

%% Continue
% Move to location under second liquid dispenser
if gui2.BSel.Value == "Bitters"
    Tr3 = Tr * transl(0.15,0,0);
    q2 = ur3.model.ikcon(Tr3);
    qWaypoint = deg2rad([55 -110 85 25 47 0]);
    QMatrixPart1 = jtraj(q1, qWaypoint, 13);
    QMatrixPart2 = jtraj(qWaypoint, q2, 12);
    QMatrix = cat(1, QMatrixPart1, QMatrixPart2);
    obstructionCheck(ur3, QMatrix, braccio, QMatrix2, BenchtopAndWall, Interrupt, linex, liney, linez, barrierFace,barrierVertex,barrierFaceNormals);
    for i = 1:25
        if gui.EditFieldMotion.Value == "Robots in motion"
            ur3.model.animate(QMatrix(i,:));
            GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassFull.model.animate(0);
            drawnow()
        else
            while gui.EditFieldMotion.Value == "Robots stopped"
                ur3.model.plot(QMatrix(i,:));
                GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
                GlassFull.model.animate(0);
            end
        end
    end
    disp('UR3: Cup is under second liquid')
    q1 = q2;

    % Collect second liquid 
    Tr4 = Tr3 * transl(0,0.05,0);
    q2 = ur3.model.ikcon(Tr4);
    QMatrix = jtraj(q1, q2, 20);
    Q1 = qbraccio;
    Q2 = deg2rad([90 0 0 0 0]);
    QMatrix2 = jtraj(Q1,Q2,20);
    obstructionCheck(ur3, QMatrix, braccio, QMatrix2, BenchtopAndWall, Interrupt, linex, liney, linez, barrierFace,barrierVertex,barrierFaceNormals);
    for i = 1:20
        if gui.EditFieldMotion.Value == "Robots in motion"
            ur3.model.animate(QMatrix(i,:));
            braccio.model.animate(QMatrix2(i,:));
            GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassFull.model.animate(0);
            drawnow()
        else
            while gui.EditFieldMotion.Value == "Robots stopped"
                ur3.model.plot(QMatrix(i,:));
                braccio.model.plot(QMatrix2(i,:));
                GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
                GlassFull.model.animate(0);
            end
        end
    end
    disp('UR3: Collected second liquid')
    disp('Braccio: Rotates to face the lime')
    QMatrix = jtraj(q2, q1, 20);
    Q1 = Q2;
    tr = Lime.model.base *  transl(0,0.05,0.06) * troty(deg2rad(180));
    Q2 = braccio.model.ikcon(tr);
    QMatrix2 = jtraj(Q1,Q2,40);
    obstructionCheck(ur3, QMatrix, braccio, QMatrix2, BenchtopAndWall, Interrupt, linex, liney, linez, barrierFace,barrierVertex,barrierFaceNormals);
    for i = 1:20
        if gui.EditFieldMotion.Value == "Robots in motion"
            ur3.model.animate(QMatrix(i,:));
            braccio.model.animate(QMatrix2(i,:));
            GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassFull.model.animate(0);
            drawnow()
        else
            while gui.EditFieldMotion.Value == "Robots stopped"
                ur3.model.plot(QMatrix(i,:));
                braccio.model.plot(QMatrix2(i,:));
                GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
                GlassFull.model.animate(0);
            end
        end
    end
    for i = 21:40
        if gui.EditFieldMotion.Value == "Robots in motion"
            braccio.model.animate(QMatrix2(i,:));
            drawnow()
        else
            while gui.EditFieldMotion.Value == "Robots stopped"
                braccio.model.plot(QMatrix2(i,:));
            end
        end
    end
    disp('UR3: Moved back down')
    disp('Braccio: Moves to collect the lime')
end
% %% Check for valid waypoint
% ur3.model.teach

%% Collecting liquid from the thrid dispenser

if gui2.SWSel.Value == "Soda water" && gui2.BSel.Value == "No bitters"
    Tr3 = Tr * transl(-0.15,0,0);
    q2 = ur3.model.ikcon(Tr3);
elseif gui2.SWSel.Value == "Soda water" && gui2.BSel.Value == "Bitters"
    Tr3 = Tr * transl(-0.3,0,0);
    q2 = ur3.model.ikcon(Tr3);
end
qWaypoint = deg2rad([55 -110 85 25 47 0]);
QMatrixPart1 = jtraj(q1, qWaypoint, 13);
QMatrixPart2 = jtraj(qWaypoint, q2, 12);
QMatrix = cat(1, QMatrixPart1, QMatrixPart2);
obstructionCheck(ur3, QMatrix, braccio, QMatrix2, BenchtopAndWall, Interrupt, linex, liney, linez, barrierFace,barrierVertex,barrierFaceNormals);
for i = 1:25
    if gui.EditFieldMotion.Value == "Robots in motion"
        ur3.model.animate(QMatrix(i,:));
        GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
        GlassFull.model.animate(0);
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robots stopped"
            ur3.model.plot(QMatrix(i,:));
            GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassFull.model.animate(0);
        end
    end
end
disp('UR3: Cup is under third liquid')
q1 = q2;

%% Collect third liquid 
Tr4 = Tr3 * transl(0,0.05,0);
q2 = ur3.model.ikcon(Tr4);
QMatrix = jtraj(q1, q2, 20);
obstructionCheck(ur3, QMatrix, braccio, QMatrix2, BenchtopAndWall, Interrupt, linex, liney, linez, barrierFace,barrierVertex,barrierFaceNormals);
if gui2.BSel.Value == "No bitters"
    Q1 = qbraccio;
    Q2 = deg2rad([90 0 0 0 0]);
    QMatrix2 = jtraj(Q1,Q2,20);
end
for i = 1:20
    if gui.EditFieldMotion.Value == "Robots in motion"
        ur3.model.animate(QMatrix(i,:));
        if gui2.BSel.Value == "No bitters"
            braccio.model.animate(QMatrix2(i,:));
        end
        GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
        GlassFull.model.animate(0);
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robots stopped"
            ur3.model.plot(QMatrix(i,:));
            if gui2.BSel.Value == "No bitters"
                braccio.model.plot(QMatrix2(i,:));
            end
            GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassFull.model.animate(0);
        end
    end
end
disp('UR3: Collected third liquid')
if gui2.BSel.Value == "No bitters"
    disp('Braccio: Rotates to face the lime')   
end

QMatrix = jtraj(q2, q1, 20);
obstructionCheck(ur3, QMatrix, braccio, QMatrix2, BenchtopAndWall, Interrupt, linex, liney, linez, barrierFace,barrierVertex,barrierFaceNormals);
if gui2.BSel.Value == "No bitters"
    Q1 = Q2;
    tr = Lime.model.base *  transl(0,0.05,0.06) * troty(deg2rad(180));
    Q2 = braccio.model.ikcon(tr);
    QMatrix2 = jtraj(Q1,Q2,40);
end
for i = 1:20
    if gui.EditFieldMotion.Value == "Robots in motion"
        ur3.model.animate(QMatrix(i,:));
        if gui2.BSel.Value == "No bitters"
            braccio.model.animate(QMatrix2(i,:));
        end
        GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
        GlassFull.model.animate(0);
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robots stopped"
            ur3.model.plot(QMatrix(i,:));
            if gui2.BSel.Value == "No bitters"
                braccio.model.plot(QMatrix2(i,:));
            end
            GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassFull.model.animate(0);
        end
    end
end
if gui2.BSel.Value == "No bitters"
    for i = 21:40
        if gui.EditFieldMotion.Value == "Robots in motion"
            braccio.model.animate(QMatrix2(i,:));
            drawnow()
        else
            while gui.EditFieldMotion.Value == "Robots stopped"
                braccio.model.plot(QMatrix2(i,:));
            end
        end
    end
end
disp('UR3: Moved back down')
if gui2.BSel.Value == "No bitters"
    disp('Braccio: Moves to collect the lime')
end


%% Continue
% Return to origin
qWaypoint = deg2rad([35 -135 110 25 85 0]);
q2 = deg2rad([0 -5 0 0 0 0]);
QMatrixPart1 = jtraj(q1, qWaypoint, 25);
QMatrixPart2 = jtraj(qWaypoint, q2, 25);
QMatrix = cat(1, QMatrixPart1, QMatrixPart2);
Q1 = deg2rad([45 0 0 0 0]);
QMatrix2 = jtraj(Q2,Q1,50);
obstructionCheck(ur3, QMatrix, braccio, QMatrix2, BenchtopAndWall, Interrupt, linex, liney, linez, barrierFace,barrierVertex,barrierFaceNormals);
for i = 1:50
    if gui.EditFieldMotion.Value == "Robots in motion"
        ur3.model.animate(QMatrix(i,:));
        braccio.model.animate(QMatrix2(i,:));
        GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
        GlassFull.model.animate(0);
        Lime.model.base = braccio.model.fkine(QMatrix2(i,:)) * transl(0,0,0.02) * troty(deg2rad(180));
        Lime.model.animate(0);
        drawnow()
    else
        while gui.EditFieldMotion.Value == "Robots stopped"
            ur3.model.plot(QMatrix(i,:));
            braccio.model.plot(QMatrix2(i,:));
            GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassFull.model.animate(0);
            Lime.model.base = braccio.model.fkine(QMatrix2(i,:)) * transl(0,0,0.02) * troty(deg2rad(180));
            Lime.model.animate(0);
        end
    end
end
disp('UR3: Returned to origin')
disp('Braccio: Lime collected and Braccio rotated to offload')

%% UR3 collecting payload from Braccio 
q1 = q2;
q2 = deg2rad([-45 -5 0 0 0 0]);
QMatrix = jtraj(q1,q2,50);
obstructionCheck(ur3, QMatrix, braccio, QMatrix2, BenchtopAndWall, Interrupt, linex, liney, linez, barrierFace,barrierVertex,barrierFaceNormals);
for i = 1:50
        if gui.EditFieldMotion.Value == "Robots in motion"
            ur3.model.animate(QMatrix(i,:));
            GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassFull.model.animate(0);
            drawnow()
        else
            while gui.EditFieldMotion.Value == "Robots stopped"
                ur3.model.plot(QMatrix(i,:));
                GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
                GlassFull.model.animate(0);
            end
        end
end
disp('UR3: Moved to collect payload from Braccio');
Q2 = deg2rad([32.4 22.5 30.6 81 0]);
QMatrix2 = jtraj(Q1,Q2,50);
obstructionCheck(ur3, QMatrix, braccio, QMatrix2, BenchtopAndWall, Interrupt, linex, liney, linez, barrierFace,barrierVertex,barrierFaceNormals);
for i = 1:50
        if gui.EditFieldMotion.Value == "Robots in motion"
            braccio.model.animate(QMatrix2(i,:));
            Lime.model.base = braccio.model.fkine(QMatrix2(i,:)) * transl(0,0,0.02) * troty(deg2rad(180));
            Lime.model.animate(0);
            drawnow()
        else
            while gui.EditFieldMotion.Value == "Robots stopped"
                braccio.model.plot(QMatrix2(i,:));
                Lime.model.base = braccio.model.fkine(QMatrix2(i,:)) * transl(0,0,0.02) * troty(deg2rad(180));
                Lime.model.animate(0);
            end
        end
end
disp('Braccio: Deposits lime into cup')
Q1 = Q2;
Q2 = deg2rad([0 15 0 0 0]);
QMatrix2 = jtraj(Q1,Q2,50);
obstructionCheck(ur3, QMatrix, braccio, QMatrix2, BenchtopAndWall, Interrupt, linex, liney, linez, barrierFace,barrierVertex,barrierFaceNormals);
for i = 1:50
        if gui.EditFieldMotion.Value == "Robots in motion"
            braccio.model.animate(QMatrix2(i,:));
            drawnow()
        else
            while gui.EditFieldMotion.Value == "Robots stopped"
                braccio.model.plot(QMatrix2(i,:));
            end
        end
end
disp('Braccio: Returns to origin')

%% UR3 trajectory for placing filled cup in payload area
q1 = q2;
q2 = deg2rad([-45 -4 0 0 -150 0]);

QMatrix = jtraj(q1,q2,50);
obstructionCheck(ur3, QMatrix, braccio, QMatrix2, BenchtopAndWall, Interrupt, linex, liney, linez, barrierFace,barrierVertex,barrierFaceNormals);
for i = 1:50
        if gui.EditFieldMotion.Value == "Robots in motion"
            ur3.model.animate(QMatrix(i,:));
            GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
            GlassFull.model.animate(0);
            Lime.model.base = ur3.model.fkine(QMatrix(i,:)) * transl(0,-0.05, 0.06);
            Lime.model.animate(0);
            drawnow()
        else
            while gui.EditFieldMotion.Value == "Robots stopped"
                ur3.model.plot(QMatrix(i,:));
                GlassFull.model.base = ur3.model.fkine(QMatrix(i,:))* transl(0,-0.1,0.06)* trotx(deg2rad(-90));
                GlassFull.model.animate(0);
                Lime.model.base = ur3.model.fkine(QMatrix(i,:)) * transl(0,-0.05, 0.06);
                Lime.model.animate(0);
            end
        end
end
disp('UR3: Moved to payload area and dropped the payload');
q1 = deg2rad([0 0 0 0 0 0]);
QMatrix = jtraj(q2,q1,50);
obstructionCheck(ur3, QMatrix, braccio, QMatrix2, BenchtopAndWall, Interrupt, linex, liney, linez, barrierFace,barrierVertex,barrierFaceNormals);
for i = 1:50
        if gui.EditFieldMotion.Value == "Robots in motion"
            ur3.model.animate(QMatrix(i,:));
            drawnow()
        else
            while gui.EditFieldMotion.Value == "Robots stopped"
                ur3.model.plot(QMatrix(i,:));
            end
        end
end
disp('UR3: Moves mack to origin')

%% GUI Control of the robot arms
while (1)
    while gui.PopUpMenu_2.Value == "Sliders"
        switch gui.PopUpMenu.Value
            case 'UR3'
                while gui.PopUpMenu.Value == "UR3"
                    qur3 = deg2rad([gui.EditField.Value, gui.EditField_2.Value, gui.EditField_3.Value, ... 
                     gui.EditField_4.Value, gui.EditField_5.Value, gui.EditField_6.Value]);
                    ur3.model.animate(qur3);
                    if gui.PopUpMenu_2.Value == "X, Y and Z Directions"
                        break;
                    end
                end
            case 'Braccio'

                while gui.PopUpMenu.Value == "Braccio"
                    qbraccio = deg2rad([gui.EditField.Value, gui.EditField_2.Value, gui.EditField_3.Value, ... 
                        gui.EditField_4.Value, gui.EditField_5.Value]);
                    braccio.model.animate(qbraccio);
                    if gui.PopUpMenu_2.Value == "X, Y and Z Directions"
                        break;
                    end
                end 
        end
    end
    while gui.PopUpMenu_2.Value == "X, Y and Z Directions"
        switch gui.PopUpMenu.Value
            case 'UR3'
                while gui.PopUpMenu.Value == "UR3"
                    tr = ur3.model.fkine(qur3) * transl (gui.EditFieldX.Value * 1.5, gui.EditFieldY.Value * 1.5,gui.EditFieldZ.Value * 1.5);
                    qur3_2 = ur3.model.ikcon(tr);
                    qMatrix = jtraj(qur3,qur3_2,3);
                    checkcollision = IsModelCollision(ur3, BenchtopAndWall,qMatrix);
                    checkcollision2 = IsModelCollision(ur3, Bowl,qMatrix);
                    checkcollision3 = IsModelCollision(ur3, GlassFull,qMatrix);
                    checkcollision4 = IsCollision(ur3.model,qMatrix,barrierFace,barrierVertex,barrierFaceNormals,false);
                    if checkcollision == 1 || checkcollision2 == 1 || checkcollision3 == 1 || checkcollision4 == 1
                        gui.EditFieldMotion.Value = "Failed pathing";
                        disp("Failed to identify safe path. Robot halting.")
                        while gui.EditFieldMotion.Value == "Failed pathing"
                            ur3.model.plot(qur3);
                        end
                    else
                        gui.EditFieldMotion.Value = "Robots in Motion";
                        for i = 1:3
                            ur3.model.animate(qMatrix(i,:));
                        end
                    end
                    qur3 = qur3_2;
                    if gui.PopUpMenu_2.Value == "Sliders"
                        break;
                    end
                end
            case 'Braccio'
                while gui.PopUpMenu.Value == "Braccio"
                    tr = braccio.model.fkine(qbraccio) * transl (gui.EditFieldX.Value, gui.EditFieldY.Value,gui.EditFieldZ.Value);
                    qbraccio_2 = braccio.model.ikcon(tr);
                    qMatrix = jtraj(qbraccio,qbraccio_2,3);
                    checkcollision = IsModelCollision(braccio, BenchtopAndWall,qMatrix);
                    checkcollision2 = IsModelCollision(braccio, Bowl,qMatrix);
                    checkcollision3 = IsModelCollision(braccio, GlassFull,qMatrix);
                    checkcollision4 = IsCollision(braccio.model,qMatrix,barrierFace,barrierVertex,barrierFaceNormals,false);
                    if checkcollision == 1 || checkcollision2 == 1 || checkcollision3 == 1 || checkcollision4 == 1 
                        gui.EditFieldMotion.Value = "Failed pathing";
                        disp("Failed to identify safe path. Robot halting.")
                        while gui.EditFieldMotion.Value == "Failed pathing"
                            braccio.model.plot(qbraccio);
                        end
                    else
                        gui.EditFieldMotion.Value = "Robots in Motion";
                        for i = 1:3
                            braccio.model.animate(qMatrix(i,:));
                        end
                    end
                    qbraccio = qbraccio_2;
                    if gui.PopUpMenu_2.Value == "Sliders"
                        break;
                    end
                end
        end
    end
end

%% Functions required for collision detection
% Some of the below functions were retrieved from lab solutions, and others
% were created from scratch using existing functions and lab solutions as a
% reference.
%% CHECK TRAJECTORY FOR COLLISIONS (JEREMY MANSFIELD 2022)
% Get Benchtop and Wall FaceNormals
function result = IsModelCollision(robot,object,qMatrix)
    NumberOfPoints = size(object.model.points{1,2},1);
    Points = [];
    for i = 1:1:NumberOfPoints
        for j = 1:1:3
            Points(i,j) = object.model.points{1,2}(i,j) + object.model.base(j,4);
        end
    end
    modelNormals = zeros(size(object.model.faces{1,2},1),3);
    for faceIndex = 1:size(object.model.faces{1,2},1)
%         v1 = object.model.points{1,2}(object.model.faces{1,2}(faceIndex,1)',:);
%         v2 = object.model.points{1,2}(object.model.faces{1,2}(faceIndex,2)',:);
%         v3 = object.model.points{1,2}(object.model.faces{1,2}(faceIndex,3)',:);
        v1 = Points(object.model.faces{1,2}(faceIndex,1)',:);
        v2 = Points(object.model.faces{1,2}(faceIndex,2)',:);
        v3 = Points(object.model.faces{1,2}(faceIndex,3)',:);
        modelNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
    end
    % Get each joints coordinates
    tr = zeros(4,4,robot.model.n+1);
    tr(:,:,1) = robot.model.base;
    L = robot.model.links;
    % For each pose, check for collisions
%     willCollide = IsCollision(robot.model,qMatrix,object.model.faces{1,2},object.model.points{1,2},modelNormals,true);
    willCollide = IsCollision(robot.model,qMatrix,object.model.faces{1,2},Points,modelNormals,true);
    if willCollide == 1
        result = true;
        return
    else
        result = false;
        return
    end

end
%% %% CHECK LINE FOR COLLISION WITH OBJECT (JEREMY MANSFIELD 2022)
% Get Benchtop and Wall FaceNormals
function result = IsModelIntersectLine(xs,ys,zs,object)

    modelNormals = zeros(size(object.model.faces{1,2},1),3);
    for faceIndex = 1:size(object.model.faces{1,2},1)
        v1 = object.model.points{1,2}(object.model.faces{1,2}(faceIndex,1)',:);
        v2 = object.model.points{1,2}(object.model.faces{1,2}(faceIndex,2)',:);
        v3 = object.model.points{1,2}(object.model.faces{1,2}(faceIndex,3)',:);
        modelNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
    end
    % For line, check for collisions
    willCollide = IsLineIntersect(xs,ys,zs,object.model.faces{1,2},object.model.points{1,2},modelNormals,true);
    if willCollide == 1
        result = true;
        return
    else
        result = false;
        return
    end

end

%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end

%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                display('Intersection');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end    
    end
end
end
%% IsLineIntersect
% Modified from IsCollision. Checks if lines intersect a model.
function result = IsLineIntersect(xs,ys,zs,faces,vertex,faceNormals,returnOnceFound)
    if nargin < 7
        returnOnceFound = true;
    end
    result = false;
    
    % Get the transform of every joint (i.e. start and end of every link)
    tr = [];
    for i = 1 : size(xs)-1
        tr(i) = transl(xs(i),ys(i),zs(i));
    end
    %tr = transl(xs(:),ys(:),zs(:));
    
% Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                display('Intersection');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end    
    end
end

%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, robot)

links = robot.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = robot.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
end
end

%% FineInterpolation
% Use results from Q2.6 to keep calling jtraj until all step sizes are
% smaller than a given max steps size
function qMatrix = FineInterpolation(q1,q2,maxStepRadians)
if nargin < 3
    maxStepRadians = deg2rad(1);
end
    
steps = 2;
while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);
end

%% InterpolateWaypointRadians
% Given a set of waypoints, finely intepolate them
function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
if nargin < 2
    maxStepRadians = deg2rad(1);
end

qMatrix = [];
for i = 1: size(waypointRadians,1)-1
    qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
end
end
%% CollisionCheck
% Checks for collisions with supplied GeneralModel parts with specified
% robot path, stalls until double press resume action with GUI.
function obstruction = obstructionCheck(robot1, qMatrix1, robot2, qMatrix2, benchtopandwallobj, interruptobj,xs,ys,zs,curtainBarrierFace,curtainBarrierVertex,curtainBarrierFaceNormals)
    disp("Checking for collisions.");
    checkCollision1 = IsModelCollision(robot1,benchtopandwallobj,qMatrix1);
    checkCollision2 = IsModelCollision(robot2,benchtopandwallobj,qMatrix2);
    checkCollision3 = IsModelCollision(robot1,interruptobj,qMatrix1);
    checkCollision4 = IsModelCollision(robot2,interruptobj,qMatrix2);
    checkCollision5 = IsCollision(robot1.model,qMatrix1,curtainBarrierFace,curtainBarrierVertex,curtainBarrierFaceNormals,false);
    checkCollision6 = IsCollision(robot2.model,qMatrix2,curtainBarrierFace,curtainBarrierVertex,curtainBarrierFaceNormals,false);
    checkCollision7 = IsModelIntersectLine(xs,ys,zs,interruptobj);
    if checkCollision1 == 1 || checkCollision2 == 1 || checkCollision3 == 1 || checkCollision4 == 1 || checkCollision5 == 1 || checkCollision6 == 1 || checkCollision7 == 1
        obstruction = true;
        gui.EditFieldMotion.Value = "Failed pathing";
        disp("Failed to identify safe path. Robot halting.")
        while gui.EditFieldMotion.Value == "Failed pathing"
            robot2.model.plot(qMatrix2);
            robot1.model.plot(qMatrix1);
        end
    else
        gui.EditFieldMotion.Value = "Robots in Motion";
%         for i = 1:3
%             braccio.model.animate(qMatrix(i,:));
%         end
        obstruction = false;
    end
    return
end
