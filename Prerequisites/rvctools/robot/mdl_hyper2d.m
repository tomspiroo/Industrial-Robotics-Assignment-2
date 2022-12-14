%MDL_HYPER2D Create model of a hyper redundant planar manipulator
%
% MDL_HYPER2D creates the workspace variable h2d which describes the
% kinematic characteristics of a serial link manipulator with 10 joints
% which at zero angles is a straight line in the XY plane.
%
% MDL_HYPER2D(N) as above but creates a manipulator with N joints.
%
% Also define the workspace vectors:
%   qz  joint angle vector for zero angle configuration
%
% R = MDL_HYPER2D(N) functional form of the above, returns the SerialLink object.
%
% [R,QZ] = MDL_HYPER2D(N) as above but also returns a vector of zero joint angles.
%
%
% Notes::
% - The manipulator in default pose is a straight line 1m long.
% - Unlike most other mdl_xxx scripts this one is actually a function that
%   behaves like a script and writes to the global workspace.
%
% See also SerialLink, mdl_hyper3d, mdl_coil, mdl_ball, mdl_twolink.

% MODEL: planar, hyper redundant, 10DOF, standard_DH

% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function [r,qq] = mdl_hyper2d(N)
    
    if nargin == 0
        N = 10;
    end
    
    % create the links
    for i=1:N
        links(i) = Link([0 0 1/N, 0]);
        q(i) = 0;
    end
    
    % and build a serial link manipulator
    robot = SerialLink(links, 'name', 'hyper2d');
    
    % place the variables into the global workspace
    if nargout == 0
        assignin('base', 'h2d', robot);
        assignin('base', 'qz', q);
    elseif nargout == 1
        r = robot;
    elseif nargout == 2
        r = robot;
        qq = q;
    end
    
    q=[-pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, pi/3, -pi/3];
    workspace = [-1 1 -1 1 -1 1];   
    scale = 0.1;
    robot.plot(q,'workspace',workspace,'scale',scale);
end
