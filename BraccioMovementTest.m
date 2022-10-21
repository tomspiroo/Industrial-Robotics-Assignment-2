clear all
close all

%% Workspace initiliasation
workspace = [-1 1 -0.1 1 -1 2];
BenchtopAndWall = GeneralModel('BenchtopAndWall','BenchtopAndWallPly.ply', transl(0,0,0), workspace);
Dispenser = GeneralModel('Dispenser','DispenserPly.ply', transl(0.2,0,0.42), workspace);
Bowl = GeneralModel('Bowl','BowlPly.ply', transl(-0.6,0.4,0), workspace);
GlassEmpty = GeneralModel('GlassEmpty','EmptyGlassPly.ply', transl(0,0.7,0), workspace);
Lime = GeneralModel('Lime','LimeSlicePly.ply', transl(-0.6,0.4,0.02), workspace);
drawnow

hold on
%% Robots and GUI initilisation
braccio = Braccio;
ur3 = UR3;
gui = GUI_App;

qbraccio = [0 deg2rad(15) 0 0 0];
qur3 = [0 0 0 0 0 0];

braccio.model.base = transl(-0.25,0.4,0);
braccio.model.plot(qbraccio);
ur3.model.base = transl(0.25,0.4,0);
ur3.model.plot(qur3);
%% Trajectory for collecting Lime
braccioQ0 = [deg2rad(180) deg2rad(70) deg2rad(10) 0 0];
braccioQ1 = qbraccio;
TrLime = Lime.model.base * trotx(deg2rad(180));
braccioQ2 = braccio.model.ikcon(TrLime,braccioQ0);
BraccioQMatrix = jtraj(braccioQ1, braccioQ2, 50);
for i = 1:50
    if gui.EditFieldMotion.Value == "Robot in motion"
        braccio.model.animate(BraccioQMatrix(i,:));
        drawnow();
    else
        while gui.EditFieldMotion.Value == "Robot stopped"
            braccio.model.plot(BraccioQMatrix(i,:));
        end
    end
end
disp('1. Lime Collected')
braccioQ1 = braccioQ2;