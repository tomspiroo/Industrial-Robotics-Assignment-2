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

QMatrix = jtraj(q1,q2,50); %Calculate Trajectory

%% CHECK TRAJECTORY FOR COLLISIONS
% Get Benchtop and Wall FaceNormals
benchFaceNormals = zeros(size(BenchtopAndWall.model.faces{1,2},1),3);
for faceIndex = 1:size(BenchtopAndWall.model.faces{1,2},1)
    v1 = BenchtopAndWall.model.points{1,2}(BenchtopAndWall.model.faces{1,2}(faceIndex,1)',:);
    v2 = BenchtopAndWall.model.points{1,2}(BenchtopAndWall.model.faces{1,2}(faceIndex,2)',:);
    v3 = BenchtopAndWall.model.points{1,2}(BenchtopAndWall.model.faces{1,2}(faceIndex,3)',:);
    benchFaceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
end
% Get each joints coordinates
tr = zeros(4,4,ur3.model.n+1);
tr(:,:,1) = ur3.model.base;
L = ur3.model.links;
% For each pose, check for collisions
willCollide = IsCollision(ur3.model,QMatrix,BenchtopAndWall.model.faces{1,2},BenchtopAndWall.model.points{1,2},benchFaceNormals,true);
display(willCollide)

%% Run movement
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

%% Functions required for collision detection
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

