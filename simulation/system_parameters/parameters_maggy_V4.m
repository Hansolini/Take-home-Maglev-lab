%{
### Parameters for the maglev system ####
The geometrical parameters are based on the actual maglev system. The
physical parameters are taken from sources online, and could be incorrect
w.r.t., the real system.
%}

%% Parameters
% Solenoids (Tuned to real solenoids and data from gikfun)
params.solenoids.x  = 0.02*[1,0,-1,0];
params.solenoids.y  = 0.02*[0,1,0,-1];
params.solenoids.r  = 0.0185/2*ones(1,4);
params.solenoids.l  = 0.012*ones(1,4);
params.solenoids.z  = params.solenoids.l/2;
params.solenoids.nw = 480;

% Permanent magnets (Tuned to the real magnets)
params.permanent.x  = sqrt(0.5*0.035^2)*[1,-1,1,-1];
params.permanent.y  = sqrt(0.5*0.035^2)*[1,1,-1,-1];
params.permanent.r  = 0.5*0.020*ones(1,4);                                     
params.permanent.l  = 2*0.0040*ones(1,4);
params.permanent.z  = params.permanent.l/2+0.2e-3; % (where they are centered)
params.permanent.J  = 1.15;

% Levitating magnet (Tuned using the "equivalent magnet" principle)
params.magnet.r     = 0.025;
params.magnet.l     = 0.0040;
params.magnet.J     = -1.1;
params.magnet.m     = 0.060; % (weight on kitchen scale, golden magnet)
params.magnet.I     = [6.1686e-06, 6.1686e-06, 1.1274e-05];
params.magnet.n     = 100;

% Sensors (7, 2, 3)
params.sensors.x  = [-0.0003, -0.0326856, 0.0130152];
params.sensors.y  = [0, 0.0137257, 0.0324254];
params.sensors.z  = [0, 0, 0];%-0.2e-3;

% Physical constants
params.physical.g   = 9.81;                                                % Gravitational acceleration [m/s^2]
params.physical.mu0 = 4*pi*1e-7;                                           % Permeability of free space (air) [N/A^2]
