function [bx,by,bz] = computeFieldTotal(x,y,z,eta,u,params,modelName)
% COMPUTEFIELDTOTAL computes the magnetic field in cartesian coordinates 
% produced by a base of permanent magnets and solenoids, defined by params,
% AND a levitating magnet defined by params and eta. The field is computed
% using the magnet/solenoid model defined by modelName.
%
% The function calculates the magnetic field components (bx, by, bz) at
% the specified points (x, y, z) in polar coordinates. u is the current in
% running through the solenoids (its size defined by the number of
% solenoids in params). modelName is either 'fast', 'accurate' or 
% 'fillament'.
%
% Example:
%   x = [0, 0, 0]; y = [0, 0, 0]; z = [0, 0.5, 1];
%   u = [1,0,-1,0]'; eta = [0,0,0.05,0,0,0,0,0,0,0,0,0]';
%   params; (from parameter file)
%   modelName = 'fast';
%   [bx,by,bz] = computeFieldTotal(x,y,z,eta,u,params,modelName);
%
% See also COMPUTEFIELDBASE, 
%          COMPUTEFORCEANDTORQUE.

% Author: Hans Alvar Engmark
% Date: 08.01.2024

%% Field from base
[bxBase,byBase,bzBase] = computeFieldBase(x,y,z,u,params,modelName);

%% Field from levitating magnet
% Relative position and rotation of points
R = rot(eta(4),eta(5),eta(6));
pRotated = inv(R)*(eta(1:3) - [x(:)';y(:)';z(:)']);

% Compute Field
I = -params.magnet.J/params.physical.mu0*params.magnet.l/2;
switch modelName
    case {'accurate','filament'}
        [bxMagnet,byMagnet,bzMagnet] = computeFieldCircularCurrentSheetCartesian(...
            pRotated(1,:),...
            pRotated(2,:),...
            pRotated(3,:),...
            params.magnet.r,...
            params.magnet.l,...
            I,...
            params.physical.mu0);

    otherwise % Default is 'fast'
        [bxMagnet,byMagnet,bzMagnet] = computeFieldCircularWireCartesian(...
            pRotated(1,:),...
            pRotated(2,:),...
            pRotated(3,:),...
            params.magnet.r,...
            I,...
            params.physical.mu0);
end
bMagnet = R*[bxMagnet; byMagnet; bzMagnet];
bxMagnet = bMagnet(1,:);
byMagnet = bMagnet(2,:);
bzMagnet = bMagnet(3,:);

%% Total field
bx = bxBase + bxMagnet;
by = byBase + byMagnet;
bz = bzBase + bzMagnet;
