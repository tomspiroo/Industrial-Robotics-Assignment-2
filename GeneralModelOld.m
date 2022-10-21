classdef GeneralModel < handle
    %MODIFIED FROM ROBOTICS TOOLBOX ROBOTCOWS
    
    properties (Constant)
        
    end
    
    properties
        model;
        location = transl(0,0,0);
        name = 'name';
        plyname;
        workspace = [-3 0.5 -1.5 1.5 -1 2];
    end
    
    methods
%% MAIN FUNCTION. CREATES GRIPPER
function self = GeneralModel(name,plyname,location,workspace)
    if 0 < nargin
        self.location = location;
        self.name = name;
        self.plyname = plyname;
        self.workspace = workspace;
    end
    self.model = self.GetModel(name,plyname);
    self.model.base = location;
    plot3d(self.model,0,'workspace',workspace,'view',[-30,30],'delay',0);
    hold on
end
    end
    
    methods (Static)
%% Get Model Function (MODIFIED FROM ROBOTICS TOOLBOX ROBOTCOWS SCRIPT)
function model = GetModel(name,plyName)
    if nargin < 1
        name = 'part';
    end
    [faceData,vertexData] = plyread(plyName,'tri');
    L1 = Link('alpha',0,'a',0,'d',0.21,'offset',0);
    model = SerialLink(L1,'name',name);
    model.faces = {faceData,[]};
    vertexData(:,2) = vertexData(:,2) + 0;
    model.points = {vertexData,[]};zz
end
    end
end