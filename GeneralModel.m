classdef GeneralModel < handle
    properties
        %> Robot model
        model;
        
        %> workspace
        workspace = [-0.6 0.6 -0.6 0.6 -0.2 1.1];   
        name = 'name';
        plyname;
        location = transl(0,0,0);
    end
    
    methods%% Class for General Model simulation
        function self = GeneralModel(name,plyname,location,workspace)
%             if 0 < nargin
%                 if length(toolModelAndTCPFilenames) ~= 2
%                     error('Please pass a cell with two strings, toolModelFilename and toolCenterPointFilename');
%                 end
%                 self.toolModelFilename = toolModelAndTCPFilenames{1};
%                 self.toolParametersFilenamure = toolModelAndTCPFilenames{2};
%             end
        if 0 < nargin
            self.location = location;
            self.name = name;
            self.plyname = plyname;
            self.workspace = workspace;
        end
        self.GetModel();
        self.PlotAndColourModel();%robot,workspace);
        drawnow
        end

        %% GetUR3Robot
        % Given a name (optional), create and return a UR3 robot model
        function GetModel(self)
            pause(0.001);
            L1 = Link('d',0,'a',0,'alpha',0,'offset',0);
            self.model = SerialLink(L1,'name',self.name);
            self.model.base = self.location;
        end
        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourModel(self)%robot,workspace)
            linkIndex = 1;
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread([self.plyname],'tri'); %#ok<AGROW>                
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end        
    end
end
