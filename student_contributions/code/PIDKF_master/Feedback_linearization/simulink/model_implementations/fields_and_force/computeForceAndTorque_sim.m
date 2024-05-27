function [Fm,Fs,taum,taus] = computeForceAndTorque_sim(x,u,params,modelName)
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
        [bx_p,by_p,bz_p,bx_s,by_s,bz_s] = computeFieldBase(p(1,:),p(2,:),p(3,:),0,params,modelName);
    
        % Compute force (permanent)
        tangent = R*[cos(repmat(theta,1,nAxial)+pi/2); sin(repmat(theta,1,nAxial)+pi/2); zeros(size(repmat(theta,1,nAxial)))];
        K_cross = K*tangent;
        F_m = zeros(3,length(params.permanent.x));

        for i = 1:length(params.permanent.r)
            F_mtot = cross(K_cross,[bx_p(1+(i-1)*nRadial*nAxial:i*nAxial*nRadial);by_p(1+(i-1)*nAxial*nRadial:i*nAxial*nRadial);bz_p(1+(i-1)*nRadial*nAxial:i*nAxial*nRadial)],1);
            Fx = reshape(F_mtot(1,:),nAxial,nRadial);
            Fy = reshape(F_mtot(2,:),nAxial,nRadial);
            Fz = reshape(F_mtot(3,:),nAxial,nRadial);
            fx = trapz(len,params.magnet.r*trapz([theta,2*pi],[Fx, Fx(:,1)],2));
            fy = trapz(len,params.magnet.r*trapz([theta,2*pi],[Fy, Fy(:,1)],2));
            fz = trapz(len,params.magnet.r*trapz([theta,2*pi],[Fz, Fz(:,1)],2));
            F_m(:,i) = [fx;fy;fz];
        end
        
        % Compute force (solenoids)
        F_s = zeros(3,length(params.solenoids.x));

        for i = 1:length(params.solenoids.r)
            F_stot = cross(K_cross,[bx_s(1+(i-1)*nRadial*nAxial:i*nRadial*nAxial);by_s(1+(i-1)*nAxial*nRadial:i*nAxial*nRadial);bz_s(1+(i-1)*nAxial*nRadial:i*nRadial*nAxial)],1);
            Fx = reshape(F_stot(1,:),nAxial,nRadial);
            Fy = reshape(F_stot(2,:),nAxial,nRadial);
            Fz = reshape(F_stot(3,:),nAxial,nRadial);
            fx = trapz(len,params.magnet.r*trapz([theta,2*pi],[Fx, Fx(:,1)],2));
            fy = trapz(len,params.magnet.r*trapz([theta,2*pi],[Fy, Fy(:,1)],2));
            fz = trapz(len,params.magnet.r*trapz([theta,2*pi],[Fz, Fz(:,1)],2));
            F_s(:,i) = [fx;fy;fz];
        end
        
        % Compute torque (permanent)
        nvec = R*[zeros(2,nRadial*nAxial); ones(1,nRadial*nAxial)];
        Jvec = K*nvec;
        T_m = zeros(3,length(params.permanent.x));
        for i = 1:length(params.permanent.r)
            T_mtot = cross(Jvec,[bx_p(1+(i-1)*nAxial*nRadial:i*nAxial*nRadial);by_p(1+(i-1)*nAxial*nRadial:i*nAxial*nRadial);bz_p(1+(i-1)*nAxial*nRadial:i*nAxial*nRadial)],1);
            Tx = reshape(T_mtot(1,:),nAxial,nRadial);
            Ty = reshape(T_mtot(2,:),nAxial,nRadial);
            Tz = reshape(T_mtot(3,:),nAxial,nRadial);
            tx = trapz(len,params.magnet.r*trapz([theta,2*pi],[Tx, Tx(:,1)],2));
            ty = trapz(len,params.magnet.r*trapz([theta,2*pi],[Ty, Ty(:,1)],2));
            tz = trapz(len,params.magnet.r*trapz([theta,2*pi],[Tz, Tz(:,1)],2));
            T_m(:,i) = [tx;ty;tz];
        end
        
        % Compute torque (solenoids)
        T_s = zeros(3,length(params.permanent.x));
        for i = 1:length(params.solenoids.r)
            T_stot = cross(Jvec,[bx_s(1+(i-1)*nAxial*nRadial:i*nAxial*nRadial);by_s(1+(i-1)*nAxial*nRadial:i*nAxial*nRadial);bz_s(1+(i-1)*nAxial*nRadial:i*nAxial*nRadial)],1);
            Tx = reshape(T_stot(1,:),nAxial,nRadial);
            Ty = reshape(T_stot(2,:),nAxial,nRadial);
            Tz = reshape(T_stot(3,:),nAxial,nRadial);
            tx = trapz(len,params.magnet.r*trapz([theta,2*pi],[Tx, Tx(:,1)],2));
            ty = trapz(len,params.magnet.r*trapz([theta,2*pi],[Ty, Ty(:,1)],2));
            tz = trapz(len,params.magnet.r*trapz([theta,2*pi],[Tz, Tz(:,1)],2));
            T_s(:,i) = [tx;ty;tz];
        end
        I = params.permanent.J/params.physical.mu0*params.permanent.l(1);
        F = [F_m F_s]*[I*ones(length(params.permanent.x),1);u];
        tau = [T_m T_s]*[I*ones(length(params.permanent.x),1);u];
        fx = F(1);
        fy = F(2);
        fz = F(3);
        tx = tau(1);
        ty = tau(2);
        tz = tau(3);




    otherwise % Default is 'fast'
        % Points along circumfrence
        theta = linspace(0,2*pi-2*pi/params.magnet.n,params.magnet.n);
        px = params.magnet.r*cos(theta);
        py = params.magnet.r*sin(theta);
        pz = zeros(size(px));
        
        R = rot(x(4),x(5),x(6));
        p = R*[px;py;pz] + x(1:3);
        
        % Compute magnetic field
%         tic
        [bx_p,by_p,bz_p,bx_s,by_s,bz_s] = computeFieldBase(p(1,:),p(2,:),p(3,:),0,params,modelName);
%         t(1) = toc;
        
        % Compute force (permanent)
        tangent = R*[cos(theta+pi/2); sin(theta+pi/2); zeros(size(theta))];
        K_cross = K*params.magnet.l*tangent;
        Fm = zeros(3,length(params.permanent.x));
        nvec = R*[zeros(2,params.magnet.n); ones(1,params.magnet.n)];
        Jvec = K*params.magnet.l*nvec;
        taum = zeros(3,length(params.permanent.x));
%         tic
        for i = 1:length(params.permanent.x)
            F_mtot = cross(K_cross,[bx_p(1+(i-1)*params.magnet.n:i*params.magnet.n);by_p(1+(i-1)*params.magnet.n:i*params.magnet.n);bz_p(1+(i-1)*params.magnet.n:i*params.magnet.n)]);
            fx = params.magnet.r*trapz([theta,2*pi],[F_mtot(1,:),F_mtot(1,1)]);
            fy = params.magnet.r*trapz([theta,2*pi],[F_mtot(2,:),F_mtot(2,1)]);
            fz = params.magnet.r*trapz([theta,2*pi],[F_mtot(3,:),F_mtot(3,1)]);
            Fm(:,i) = [fx;fy;fz];
            T_mtot = cross(Jvec,[bx_p(1+(i-1)*params.magnet.n:i*params.magnet.n);by_p(1+(i-1)*params.magnet.n:i*params.magnet.n);bz_p(1+(i-1)*params.magnet.n:i*params.magnet.n)]);
            tx = params.magnet.r*trapz([theta,2*pi],[T_mtot(1,:),T_mtot(1,1)]);
            ty = params.magnet.r*trapz([theta,2*pi],[T_mtot(2,:),T_mtot(2,1)]);
            tz = params.magnet.r*trapz([theta,2*pi],[T_mtot(3,:),T_mtot(3,1)]);
            taum(:,i) = [tx;ty;tz];
        end
%         t(2)=toc;
        % Compute force (solenoids)
        Fs = zeros(3,length(params.solenoids.x));
        taus = zeros(3,length(params.solenoids.x));
%         tic
        for i = 1:length(params.solenoids.x)
            F_stot = cross(K_cross,[bx_s(1+(i-1)*params.magnet.n:i*params.magnet.n);by_s(1+(i-1)*params.magnet.n:i*params.magnet.n);bz_s(1+(i-1)*params.magnet.n:i*params.magnet.n)]);
            fx = params.magnet.r*trapz([theta,2*pi],[F_stot(1,:),F_stot(1,1)]);
            fy = params.magnet.r*trapz([theta,2*pi],[F_stot(2,:),F_stot(2,1)]);
            fz = params.magnet.r*trapz([theta,2*pi],[F_stot(3,:),F_stot(3,1)]);
            Fs(:,i) = [fx;fy;fz];
            T_stot = cross(Jvec,[bx_s(1+(i-1)*params.magnet.n:i*params.magnet.n);by_s(1+(i-1)*params.magnet.n:i*params.magnet.n);bz_s(1+(i-1)*params.magnet.n:i*params.magnet.n)]);
            tx = params.magnet.r*trapz([theta,2*pi],[T_stot(1,:),T_stot(1,1)]);
            ty = params.magnet.r*trapz([theta,2*pi],[T_stot(2,:),T_stot(2,1)]);
            tz = params.magnet.r*trapz([theta,2*pi],[T_stot(3,:),T_stot(3,1)]);
            taus(:,i) = [tx;ty;tz];
        end
%         t(3) = toc;
        % Compute torque (permanent)
%         nvec = R*[zeros(2,params.magnet.n); ones(1,params.magnet.n)];
%         Jvec = K*params.magnet.l*nvec;
%         taum = zeros(3,length(params.permanent.x));
%         tic;
%         for i = 1:length(params.permanent.x)
%             T_mtot = cross(Jvec,[bx_p(1+(i-1)*params.magnet.n:i*params.magnet.n);by_p(1+(i-1)*params.magnet.n:i*params.magnet.n);bz_p(1+(i-1)*params.magnet.n:i*params.magnet.n)]);
%             tx = params.magnet.r*trapz([theta,2*pi],[T_mtot(1,:),T_mtot(1,1)]);
%             ty = params.magnet.r*trapz([theta,2*pi],[T_mtot(2,:),T_mtot(2,1)]);
%             tz = params.magnet.r*trapz([theta,2*pi],[T_mtot(3,:),T_mtot(3,1)]);
%             taum(:,i) = [tx;ty;tz];
%         end
%         t(4)=toc;
        % Compute torque (solenoids)
%         taus = zeros(3,length(params.solenoids.x));
%         tic;
%         for i = 1:length(params.permanent.x)
%             T_stot = cross(Jvec,[bx_s(1+(i-1)*params.magnet.n:i*params.magnet.n);by_s(1+(i-1)*params.magnet.n:i*params.magnet.n);bz_s(1+(i-1)*params.magnet.n:i*params.magnet.n)]);
%             tx = params.magnet.r*trapz([theta,2*pi],[T_stot(1,:),T_stot(1,1)]);
%             ty = params.magnet.r*trapz([theta,2*pi],[T_stot(2,:),T_stot(2,1)]);
%             tz = params.magnet.r*trapz([theta,2*pi],[T_stot(3,:),T_stot(3,1)]);
%             taus(:,i) = [tx;ty;tz];
%         end
%         t(5)=toc;

%         I = params.permanent.J/params.physical.mu0*params.permanent.l(1);
%         F = [F_m F_s]*[I*ones(length(params.permanent.x),1);u];
%         tau = [T_m T_s]*[I*ones(length(params.permanent.x),1);u];
%         fx = F(1);
%         fy = F(2);
%         fz = F(3);
%         tx = tau(1);
%         ty = tau(2);
%         tz = tau(3);
end

