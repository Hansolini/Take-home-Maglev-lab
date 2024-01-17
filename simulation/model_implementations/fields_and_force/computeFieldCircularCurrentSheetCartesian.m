function [bx,by,bz] = computeFieldCircularCurrentSheetCartesian(x,y,z,r,l,I,mu0)
% COMPUTEFIELDCIRCULARCURRENTSHEETCARTESIAN computes the magnetic field in 
% cartesian coordinates produced by a cylindrical current carrying sheet 
% (an ideal solenoid). The current sheet is centered at the origin, with 
% its main axis aligned with the z-axis.
%
% The function calculates the magnetic field components (bx, by, bz) at
% the specified points (x, y, z) in polar coordinates. The strength and
% direction of the magnetic field is determined by the radius r of current 
% sheet, the current I running through it and the magnetic permeability of 
% the medium (e.g., air).
%
% Example:
%   x = [0, 0, 0]; y = [0, 0, 0]; z = [0, 0.5, 1];
%   r = 1; I = 1; mu0 = 4*pi*1e-7;
%   [bphi, brho, bz] = computeFieldCircularCurrentSheetCartesian(x,y,z,r,I,mu0);
%
% See also COMPUTEFIELDCIRCULARCURRENTSHEETPOLAR, 
%          COMPUTEFIELDCIRCULARWIRECARTESIAN.

% COMPUTEFIELDCIRCULARCURRENTSHEETCARTESIAN uses the implementation described in [1].

% Author: Hans Alvar Engmark
% Date: 08.01.2024

% References:
% [1] Engmark, Hans Alvar, and Kiet Tuan Hoang. 
%     "Modeling and Control of a Magnetic Levitation Platform." 
%     IFAC-PapersOnLine 56.2 (2023): 7276-7281.

% Convert input to polar coordinates
[phi,rho,z] = cart2pol(x,y,z);

% Compute field
[bPhi,bRho,bz] = computeFieldCircularCurrentSheetPolar(phi,rho,z,r,l,I,mu0);

% Convert result to Cartesian coordinates
[bx,by,bz] = pol2cart(bPhi,bRho,bz);
