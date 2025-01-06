function [b,e]=besolv(Z,U,At,A,D,L,g,r);
% BESOLV Low level DSR function.
%        [B,E]=besolv(Z,U,At,A,D,L,g,r);
%        Optimal least squares metod to solve
%        Z = tilde(B) U for system matrices B and E
%        ALGORITHM
%        Solve equivalent system cs([B;E])=N\cs(Z)

