function [bphi,brho,bz] = computeFieldCircularWirePolar(phi,rho,z,r,I,mu0)
% COMPUTEFIELDCIRCULARWIREPOLAR computes the magnetic field in polar 
% coordinates produced by a single circular current carrying wire.
% The wire is centered at the origin and lies parallel to the xy-plane.
%
% The function calculates the magnetic field components (bphi, brho, bz) at
% the specified points (phi, rho, z) in polar coordinates. The strength and
% direction of the magnetic field is determined by the radius r of the wire 
% loop, the current I running through it and the magnetic permeability of 
% the medium (e.g., air).
%
% Example:
%   phi = [0, pi/4, pi/2]; rho = [1, 1.5, 2]; z = [0, 0.5, 1];
%   r = 1; I = 1; mu0 = 4*pi*1e-7;
%   [bphi, brho, bz] = computeFieldCircularWirePolar(phi,rho,z,r,I,mu0);
%
% See also COMPUTEFIELDCIRCULARWIRECARTESIAN, 
%          COMPUTEFIELDCIRCULARCURRENTSHEETPOLAR.

% COMPUTEFIELDCIRCULARWIREPOLAR uses the implementation described in [1].

% Author: Hans Alvar Engmark
% Date: 08.01.2024

% References:
% [1] Engmark, Hans Alvar, and Kiet Tuan Hoang. 
%     "Modeling and Control of a Magnetic Levitation Platform." 
%     IFAC-PapersOnLine 56.2 (2023): 7276-7281.

%% rho ~= 0 (2a)
c = mu0*I./(4*pi*sqrt(r.*rho));

k2 = 4*r*rho./((r+rho).^2+z.^2);
k2 = min(k2,1); k2 = max(k2,0); % Fix for numerical error
[K,E] = ellipke(k2);

bphi = phi;
brho = -(z./rho).*c.*sqrt(k2).*(K-(rho.^2+r^2+z.^2)./((rho-r).^2+z.^2).*E);
bz   =            c.*sqrt(k2).*(K-(rho.^2-r^2+z.^2)./((rho-r).^2+z.^2).*E);

%% rho = 0 (2b)
tol = 1e-8; % Fix for numerical error
indices = abs(rho) < tol;

bphi(indices) = 0;
brho(indices) = 0;
bz(indices)   = mu0*r^2*I./(2*(r^2+z(indices).^2).^(3/2)); % Might be something wrong going on here
