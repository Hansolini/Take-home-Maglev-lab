function [bx,by,bz] = computeFieldBase(x,y,z,u,params,modelName)
% COMPUTEFIELDBASE computes the magnetic field in cartesian coordinates 
% produced by a base of permanent magnets and solenoids, defined by params,
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
%   u = [1,0,-1,0]';
%   params; (from parameter file)
%   modelName = 'fast';
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
switch modelName
    case {'filament','accurate'}
        for i = 1:length(params.permanent.r)
            I = params.permanent.J/params.physical.mu0*params.permanent.l(i);
            
            [bxTemp,byTemp,bzTemp] = computeFieldCircularCurrentSheetCartesian(...
                x-params.permanent.x(i),...
                y-params.permanent.y(i),...
                z-params.permanent.z(i),...
                params.permanent.r(i),...
                params.permanent.l(i),...
                I,...
                params.physical.mu0);
            
            bx = bx + bxTemp;
            by = by + byTemp;
            bz = bz + bzTemp;
        end

    otherwise % Default is 'fast'
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
end

%% Field from solenoids
switch modelName
    case 'filament'
        for i = 1:length(params.solenoids.r)
            I = u(i);

            % Computing number of windings in axial and radial directions
            ratio = sqrt(params.solenoids.nw/(params.solenoids.l(i)*params.solenoids.r(i)));
            nwAxial  = floor(params.solenoids.l(i)*ratio);
            nwRadial = round(params.solenoids.nw/nwAxial);
            
            % Computing filament offset position
            deltaZ = linspace(-params.solenoids.l(i)/2,params.solenoids.l(i)/2,nwAxial);
            deltaR = linspace(0,params.solenoids.r(i),nwRadial+1);

            % Looping over each filament
            for j = 1:nwAxial
                for k = 2:nwRadial % Looping from 2 to 'skip' radius=0 (also added one extra in the radial direction to compensate)
                    [bxTemp,byTemp,bzTemp] = computeFieldCircularWireCartesian(...
                        x-params.solenoids.x(i),...
                        y-params.solenoids.y(i),...
                        z-params.solenoids.z(i)-deltaZ(j),...
                        deltaR(k),...
                        I,...
                        params.physical.mu0);
                    
                    bx = bx + bxTemp;
                    by = by + byTemp;
                    bz = bz + bzTemp;
                end
            end
        end

    case 'accurate'
        % TODO: Need correct multiplication when using current sheets
        for i = 1:length(params.solenoids.r)
            I = u(i);
        
            [bxTemp,byTemp,bzTemp] = computeFieldCircularCurrentSheetCartesian(...
                x-params.solenoids.x(i),...
                y-params.solenoids.y(i),...
                z-params.solenoids.z(i),...
                params.solenoids.r(i),...
                params.solenoids.l(i),...
                I,...
                params.physical.mu0);
            
            bx = bx + bxTemp*params.solenoids.nw;
            by = by + byTemp*params.solenoids.nw;
            bz = bz + bzTemp*params.solenoids.nw;
        end

    otherwise % Default is 'fast'
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
end
