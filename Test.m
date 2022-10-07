%% Test Robot
robot = Braccio;
q = [0 0 0 0 0];
workspace = [-1 1 0 0.8 -1 1];
scale = 1;
robot.model.base = transl(-0.25,0.4,0);
robot.model.plot(q,'workspace',workspace,'scale',scale);
%robot.model.teach();
hold on;
%% Create Workspace
% Create Tabletop Surface
ttPoint = [0,0,0];
ttNormal = [0,-1,0];
ttvertx = [1,-1;1,-1];
ttverty = [0,0;0.8,0.8];
ttvertz = [0,0;0,0];
tabletop_h = surf(ttvertx,ttverty,ttvertz);