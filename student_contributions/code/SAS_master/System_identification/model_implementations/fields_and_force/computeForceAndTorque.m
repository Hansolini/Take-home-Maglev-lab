function [fx,fy,fz,tx,ty,tz] = computeForceAndTorque(x,u,params,modelName)
% COMPUTEFORCEANDTORQUE computes the magnetic force and torque produced by 
% a base of permanent magnets and solenoids, defined by params,
% on a levitating magnet defined by params and x. The force is computed
% using the magnet/solenoid model defined by modelName.
%
% u is the current in running through the solenoids (its size defined by 
% the number of solenoids in params). modelName is either 'fast', 
% 'accurate' or 'fillament'.
%
% Example:
%   x = [0,0,0.05,0,0,0,0,0,0,0,0,0]'; u = [1,0,-1,0]'; 
%   params; (from parameter file)
%   modelName = 'fast';
%   [fx,fy,fz,tx,ty,tz] = computeForceAndTorque(x,u,params,modelName);
%
% See also COMPUTEFIELDBASE.

% Author: Hans Alvar Engmark
% Date: 08.01.2024

% Additional parameters
K  = -params.magnet.J/params.physical.mu0;

switch modelName
    case {'accurate', 'filament'}
        % Defining number of points to evaluate on the magnet surface
        % (increase for better accuracy)
        nRadial = 100;
        nAxial  = 21; % This number seems to be causing some asymmetry in the force and torque when not odd

        % Points on surface
        theta = linspace(0,2*pi-2*pi/nRadial,nRadial);
        len = linspace(-params.magnet.l/2,params.magnet.l/2,nAxial);

        px = repmat(params.magnet.r*cos(theta),1,nAxial);
        py = repmat(params.magnet.r*sin(theta),1,nAxial);
        pz = repmat(len,1,nRadial);
    
        R = rot(x(4),x(5),x(6));
        p = R*[px;py;pz] + x(1:3);
        
        % Compute magnetic field
        [bx,by,bz] = computeFieldBase(p(1,:),p(2,:),p(3,:),u,params,modelName);
    
        % Compute force 
        tangent = R*[cos(repmat(theta,1,nAxial)+pi/2); sin(repmat(theta,1,nAxial)+pi/2); zeros(size(repmat(theta,1,nAxial)))];
        F = cross(K*tangent,[bx;by;bz],1);
    
        Fx = reshape(F(1,:),nAxial,nRadial);
        Fy = reshape(F(2,:),nAxial,nRadial);
        Fz = reshape(F(3,:),nAxial,nRadial);

        fx = trapz(len,params.magnet.r*trapz([theta,2*pi],[Fx, Fx(:,1)],2));
        fy = trapz(len,params.magnet.r*trapz([theta,2*pi],[Fy, Fy(:,1)],2));
        fz = trapz(len,params.magnet.r*trapz([theta,2*pi],[Fz, Fz(:,1)],2));
        
        % Compute torque
        nvec = R*[zeros(2,nRadial*nAxial); ones(1,nRadial*nAxial)];
        T = cross(K*nvec,[bx;by;bz]);

        Tx = reshape(T(1,:),nAxial,nRadial);
        Ty = reshape(T(2,:),nAxial,nRadial);
        Tz = reshape(T(3,:),nAxial,nRadial);
        
        tx = trapz(len,params.magnet.r*trapz([theta,2*pi],[Tx, Tx(:,1)],2));
        ty = trapz(len,params.magnet.r*trapz([theta,2*pi],[Ty, Ty(:,1)],2));
        tz = trapz(len,params.magnet.r*trapz([theta,2*pi],[Tz, Tz(:,1)],2));

    otherwise % Default is 'fast'
        % Points along circumfrence
        theta = linspace(0,2*pi-2*pi/params.magnet.n,params.magnet.n);
        px = params.magnet.r*cos(theta);
        py = params.magnet.r*sin(theta);
        pz = zeros(size(px));
        
        R = rot(x(4),x(5),x(6));
        p = R*[px;py;pz] + x(1:3);
        
        % Compute magnetic field
        [bx,by,bz] = computeFieldBase(p(1,:),p(2,:),p(3,:),u,params,modelName);
        
        % Compute force 
        tangent = R*[cos(theta+pi/2); sin(theta+pi/2); zeros(size(theta))];
        F = cross(K*params.magnet.l*tangent,[bx;by;bz]);
        
        fx = params.magnet.r*trapz([theta,2*pi],[F(1,:),F(1,1)]);
        fy = params.magnet.r*trapz([theta,2*pi],[F(2,:),F(2,1)]);
        fz = params.magnet.r*trapz([theta,2*pi],[F(3,:),F(3,1)]);
        
        % Compute torque
        nvec = R*[zeros(2,params.magnet.n); ones(1,params.magnet.n)];
        T = cross(K*params.magnet.l*nvec,[bx;by;bz]);
        
        tx = params.magnet.r*trapz([theta,2*pi],[T(1,:),T(1,1)]);
        ty = params.magnet.r*trapz([theta,2*pi],[T(2,:),T(2,1)]);
        tz = params.magnet.r*trapz([theta,2*pi],[T(3,:),T(3,1)]);
end
