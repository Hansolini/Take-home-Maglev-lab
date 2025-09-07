function y = maglevSystemMeasurements(x,u,params,modelName)
% MAGLEVSYSTEMMEASUREMENTS implements the function h in the ODE 
%   dxdt = f(x,u); 
%      y = h(x,u);
% defining the sensor measurements of a magnetic levitation system. The 
% system is fully defined by the params struct, and the magnetic model to 
% be used for the measurements is defined by modelName, which can be either
% 'fast', 'accurate' or 'filament'. The measurements computes the effect
% from all magnetic components of the system.
%
% Example:
%   params; (from parameter file)
%   modelName = 'fast';
%   x = [0,0,0.1,0,0,0,0,0,0,0,0,0]';
%   u = [1,0,-1,0]';
%
%   h = @(x,u) maglevSystemMeasuremets(x,u,params,modelName);
%   y = h(x,u);
%
% See also MAGLEVSYSTEMDYNAMICS,
%          COMPUTEFIELDTOTAL.

% MAGLEVSYSTEMMEASUREMENTS uses the implementation described in [1].

% Author: Hans Alvar Engmark
% Date: 08.01.2024

% References:
% [1] Engmark, Hans Alvar, and Kiet Tuan Hoang. 
%     "Modeling and Control of a Magnetic Levitation Platform." 
%     IFAC-PapersOnLine 56.2 (2023): 7276-7281.

[bx,by,bz] = computeFieldTotal(params.sensors.x,params.sensors.y,params.sensors.z,x,u,params,modelName);
y = reshape([bx(:)'; by(:)'; bz(:)'],3*numel(bx),1);
