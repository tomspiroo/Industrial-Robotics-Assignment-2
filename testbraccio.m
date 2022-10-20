robot = Braccio;
qrobot = [0 0 0 0 0];
workspace = [-1 1 0 0.8 -1 1];
scale = 1;
robot.model.base = transl(-0.25,0.4,0);
robot.model.plot(qrobot,'workspace',workspace,'scale',scale,'nojoints');
robot.model.teach