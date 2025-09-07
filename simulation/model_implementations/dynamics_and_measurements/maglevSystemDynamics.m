function dx = maglevSystemDynamics(x,u,params,modelName)
% MAGLEVSYSTEMDYNAMICS implements the function f in the ODE dxdt = f(x,u)
% defining the dynamics of a magnetic levitation system. The system is
% fully defined by the params struct, and the magnetic model to be used is
% defined by modelName, which can be either 'fast', 'accurate' or 
% 'filament'.
%
% Example:
%   params; (from parameter file)
%   modelName = 'fast';
%   u = @(t) ...; (user defined)
%   f = @(t,x) maglevSystemDynamics(x,u(t),params,modelName);
%
%   t = [0, 10]; x0 = [0,0,0.1,0,0,0,0,0,0,0,0,0]';
%   [t,x] = ode15s(f,t,x0);
%
% See also MAGLEVSYSTEMMEASUREMENTS.

% MAGLEVSYSTEMDYNAMICS uses the implementation described in [1].

% Author: Hans Alvar Engmark
% Date: 08.01.2024

% References:
% [1] Engmark, Hans Alvar, and Kiet Tuan Hoang. 
%     "Modeling and Control of a Magnetic Levitation Platform." 
%     IFAC-PapersOnLine 56.2 (2023): 7276-7281.

% Computing force and torque on levitating magnet
[fx,fy,fz,tx,ty,tz] = computeForceAndTorque(x,u,params,modelName);

% Setting up system matrices
A = [
    zeros(6), eye(6);
    zeros(6), zeros(6)
    ];

B = [
    zeros(6);
    eye(6)
    ];

% Mass and inertia properties of the magnet
M = [
    params.magnet.m*eye(3), zeros(3);
    zeros(3), diag(params.magnet.I)
    ];

% Computing the nonlinear function f(x,u)
f = M\([fx;fy;fz;tx;ty;tz]-[zeros(3,1);cross(x(10:12),diag(params.magnet.I)*x(10:12))])...
    -[zeros(2,1);params.physical.g;zeros(3,1)];

dx = A*x+B*f;
