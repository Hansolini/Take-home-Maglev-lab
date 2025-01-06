function [yp,Vpe,xp] = dsropt(a,b,d,e,cf,f,Y,U,x0);
% DSROPT Compute optimal predictions based on model matrices
%        computed by DSR, e.g., as [A,B,D,E,CF,F,x0]=dsr(Y,U,L);
%   
%   [Yp,Vpe,Xp] = dsropt(A,B,D,E,CF,F,Y,U,x0);
%   ALGORITHM:
%   The innovations form model, (see e.g. help dsr), can be written 
%   as on optimal prediction (yp_t) of the output vector (y_t), i.e.
%
%   x_{t+1} = A x_t + B u_t + C (y_t - (D x_t + E u_t))
%   yp_t    = D x_t + E u_t
%
%   where C=CF*inv(F) is the Kalman filter gain matrix. 
%
%   x_{t+1} = (A - C D) x_t + (B - C E) u_t + C y_t
%   yp_t    = D x_t + E u_t
%
%   The optimal prediction (yp_t) of the output (y_t) uses 
%   all past outputs up to time (t-1), i.e. (...y_{t-2},y_{t-1}) and
%   all past inputs up to time (t-1) (when E=0), i.e. (...,u_{t-1}).
%   If E .neq. 0 then also the present input u_t is used.
% 
%   ON INPUT:
%   A B D E CF F - Innovations form model matrices as computed by DSR
%   x0           - (n x 1) vector of initial values, computed by DSR.
%   ON OUTPUT
%   Yp           - Matrix with optimal predictions, i.e. a (N x m) matrix.
%   Vpe          - Prediction error criterion.
%   Xp           - Matrix with predicted states, i.e. a (N x n) array.
%-------------------------------------------------------------------------

% WRITTEN TO BE USED AS SUPPLEMENT TO THE DSR IDENTIFICATION ALGORITHM
% DATE: 28. august 1996, David Di Ruscio
% FUNCTIONS CALLED: DSRSIM

%.1


