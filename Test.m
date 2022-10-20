%% Test Robot
robot = Braccio;
ur3 = UR3;
qrobot = [0 0 0 0 0];
qur3 = [0 0 0 0 0 0];
workspace = [-1 1 0 0.8 -1 1];
scale = 1;
robot.model.base = transl(-0.25,0.4,0);
robot.model.plot(qrobot,'workspace',workspace,'scale',scale);
%robot.model.teach();
hold on;
ur3.model.base = transl(0.25,0.4,0);
ur3.model.plot(qur3,'workspace',workspace,'scale',scale);
%% Create Workspace
% Create Tabletop Surface
ttPoint = [0,0,0];
ttNormal = [0,0,-1];
ttvertx = [1,-1;1,-1];
ttverty = [0,0;0.8,0.8];
ttvertz = [0,0;0,0];
tabletop_h = surf(ttvertx,ttverty,ttvertz);

% Create Wall Surface
wallPoint = [0,0,0];
wallNormal = [0,-1,0];
wallvertx = [1,-1;1,-1];
wallverty = [0,0;0,0];
wallvertz = [-1,-1;1,1];
wall_h = surf(wallvertx,wallverty,wallvertz);