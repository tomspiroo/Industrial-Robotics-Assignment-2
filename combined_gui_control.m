close all
clear all

set(0,'DefaultFigureWindowStyle','docked')
robot = Braccio;
ur3 = UR3;
gui = GUI_App;

qrobot = [0 0 0 0 0];
qur3 = [0 0 0 0 0 0];
workspace = [-1 1 -1 1 -1 1];
scale = 0.5;
robot.model.base = transl(-0.25,0.4,0);
robot.model.plot(qrobot, 'workspace',workspace,'scale',scale, 'nojoints');
hold on;
ur3.model.base = transl(0.25,0.4,0);
ur3.model.plot(qur3,'workspace',workspace,'scale',scale);

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
                    qrobot = deg2rad([gui.EditField.Value, gui.EditField_2.Value, gui.EditField_3.Value, ... 
                        gui.EditField_4.Value, gui.EditField_5.Value]);
                    robot.model.animate(qrobot);
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
                    tr = ur3.model.fkine(qur3) * transl (gui.EditFieldX.Value, gui.EditFieldY.Value,gui.EditFieldZ.Value);
                    qur3_2 = ur3.model.ikcon(tr);
                    ur3.model.animate(qur3_2);
                    qur3 = qur3_2;
                    if gui.PopUpMenu_2.Value == "Sliders"
                        break;
                    end
                end
            case 'Braccio'
                while gui.PopUpMenu.Value == "Braccio"
                    tr = robot.model.fkine(qrobot) * transl (gui.EditFieldX.Value, gui.EditFieldY.Value,gui.EditFieldZ.Value);
                    qrobot_2 = robot.model.ikcon(tr);
                    robot.model.animate(qrobot_2);
                    qrobot = qrobot_2;
                    if gui.PopUpMenu_2.Value == "Sliders"
                        break;
                    end
                end
        end
    end
end

%% Combined gui control with collision check
close all
clear all

set(0,'DefaultFigureWindowStyle','docked')
braccio = Braccio;
ur3 = UR3;
gui = GUI_App;

qbraccio = [0 0 0 0 0];
qur3 = [0 0 0 0 0 0];
workspace = [-1 1 -0.1 1 -1 2];
BenchtopAndWall = GeneralModel('BenchtopAndWall','BenchtopAndWallPly.ply', transl(0,0,0), workspace);
Bowl = GeneralModel('Bowl','BowlPly.ply', transl(-0.25,0.7,0), workspace);
GlassFull = GeneralModel('GlassFull','FullglassPly.ply', transl(0.25,0.7,0), workspace);
scale = 0.5;
braccio.model.base = transl(-0.25,0.4,0);
braccio.model.plot(qbraccio, 'workspace',workspace,'scale',scale, 'nojoints');
hold on;
ur3.model.base = transl(0.25,0.4,0);
ur3.model.plot(qur3,'workspace',workspace,'scale',scale);

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
                    tr = ur3.model.fkine(qur3) * transl (gui.EditFieldX.Value, gui.EditFieldY.Value,gui.EditFieldZ.Value);
                    qur3_2 = ur3.model.ikcon(tr);
                    qMatrix = jtraj(qur3,qur3_2,3);
                    checkcollision = IsModelCollision(ur3, BenchtopAndWall,qMatrix);
                    checkcollision2 = IsModelCollision(ur3, Bowl,qMatrix);
                    checkcollision3 = IsModelCollision(ur3, GlassFull,qMatrix);
                    if checkcollision == 1 || checkcollision2 == 1 || checkcollision3 == 1 
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
                    if checkcollision == 1 || checkcollision2 == 1 || checkcollision3 == 1 
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

    modelNormals = zeros(size(object.model.faces{1,2},1),3);
    for faceIndex = 1:size(object.model.faces{1,2},1)
        v1 = object.model.points{1,2}(object.model.faces{1,2}(faceIndex,1)',:);
        v2 = object.model.points{1,2}(object.model.faces{1,2}(faceIndex,2)',:);
        v3 = object.model.points{1,2}(object.model.faces{1,2}(faceIndex,3)',:);
        modelNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
    end
    % Get each joints coordinates
    tr = zeros(4,4,robot.model.n+1);
    tr(:,:,1) = robot.model.base;
    L = robot.model.links;
    % For each pose, check for collisions
    willCollide = IsCollision(robot.model,qMatrix,object.model.faces{1,2},object.model.points{1,2},modelNormals,true);
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

