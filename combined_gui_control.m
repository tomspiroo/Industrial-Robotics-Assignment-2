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