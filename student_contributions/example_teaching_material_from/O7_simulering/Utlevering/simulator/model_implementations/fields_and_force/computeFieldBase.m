function [bx,by,bz] = computeFieldBase(x,y,z,u,params)
% COMPUTEFIELDBASE computes the magnetic field in cartesian coordinates 
% produced by a base of permanent magnets and solenoids, defined by params.
%
% The function calculates the magnetic field components (bx, by, bz) at
% the specified points (x, y, z) in polar coordinates. u is the current in
% running through the solenoids (its size defined by the number of
% solenoids in params).
%
% Example:
%   x = [0, 0, 0]; y = [0, 0, 0]; z = [0, 0.5, 1];
%   u = [1,0,-1,0]';
%   params; (from parameter file)
%   [bx,by,bz] = computeFieldBase(x,y,z,u,params,modelName);
%
% See also COMPUTEFIELDTOTAL, 
%          COMPUTEFORCEANDTORQUE.

% Author: Hans Alvar Engmark
% Date: 08.01.2024

% Initialize field array
bx = zeros(size(x));
by = zeros(size(y));
bz = zeros(size(z));

%% Field from permanent magnets
for i = 1:length(params.permanent.r)
    I = params.permanent.J/params.physical.mu0*params.permanent.l(i);
    
    [bxTemp,byTemp,bzTemp] = computeFieldCircularWireCartesian(...
        x-params.permanent.x(i),...
        y-params.permanent.y(i),...
        z-params.permanent.z(i),...
        params.permanent.r(i),...
        I,...
        params.physical.mu0);
    
    bx = bx + bxTemp;
    by = by + byTemp;
    bz = bz + bzTemp;
end

%% Field from solenoids
for i = 1:length(params.solenoids.r)
    I = u(i);

    [bxTemp,byTemp,bzTemp] = computeFieldCircularWireCartesian(...
        x-params.solenoids.x(i),...
        y-params.solenoids.y(i),...
        z-params.solenoids.z(i),...
        params.solenoids.r(i),...
        I,...
        params.physical.mu0);
    
    bx = bx + bxTemp*params.solenoids.nw;
    by = by + byTemp*params.solenoids.nw;
    bz = bz + bzTemp*params.solenoids.nw;
end

