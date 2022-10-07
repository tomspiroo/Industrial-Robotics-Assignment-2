classdef Braccio < handle
    properties
        %> Robot model
        model;
        
        %> workspace
        workspace = [-2 1 -1 1 -1.01 1];   
      
    end
    
    methods%% Class for Braccio robot simulation
        function self = Braccio(toolModelAndTCPFilenames)
            if 0 < nargin
                if length(toolModelAndTCPFilenames) ~= 2
                    error('Please pass a cell with two strings, toolModelFilename and toolCenterPointFilename');
                end
                self.toolModelFilename = toolModelAndTCPFilenames{1};
                self.toolParametersFilenamure = toolModelAndTCPFilenames{2};
            end
            
            self.GetBraccioRobot();
            %self.PlotAndColourRobot();%robot,workspace);

            drawnow
        end

        %% GetUR3Robot
        % Given a name create and return a Braccio robot model
        function GetBraccioRobot(self)
            pause(0.001);
            name = 'Arduino Braccio';  
            L(1) = Link('d', 0.0715, 'a', 0, 'alpha', -1.5708,'offset',0);
            L(2) = Link('d', 0, 'a', -0.125, 'alpha', 0, 'offset', 1.5708);
            L(3) = Link('d', 0, 'a', -0.125, 'alpha', 0,'offset',0);
            L(4) = Link('d', 0, 'a', -0.125, 'alpha', 1.5708,'offset',-1.5708);
            L(5) = Link('d', 0.192, 'a', 0, 'alpha', 0,'offset',0);

            % For point cloud and for brick collection.
            L(1).qlim = deg2rad([0 180]);
            L(2).qlim = deg2rad([15 165]);
            L(3).qlim = deg2rad([0 180]);
            L(4).qlim = deg2rad([0 180]);
            L(5).qlim = deg2rad([0 180]);

            

             
            self.model = SerialLink(L,'name',name);
        end
    end
end

