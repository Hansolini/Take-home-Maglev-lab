function [a,b,d,e,c,f,x0,E_J1,DXJ,sn]=dsr_e(y,u,L,g,J,n,im)
%DSR_e  Deterministic and Stochastic system identification and Realization
%       for open and closed loop systems.
%       [A,B,D,E,K,F,x0,Ef1,Yp]=dsr_e(Y,U,L,g,J,n)
%       PURPOSE:
%       Estimate the the matrices (A,B,D,E,K,F)
%       in the discrete time combined deterministic and 
%       stochastic dynamic model on innovations form 
%       and the initial state, x0.
%
% ON INPUT:
%       Y         - Output time series matrix of size (N x m) 
%                   where N is the number of observations and
%                   m is the number of output variables.
%       U         - Input time series matrix of size (N x r)
%                   where r is the number of input variables.
%       L         - Number of block rows in extended observability matrix.
%                   Choose L .geq. 1. This means that one can estimate
%                   system order (n) bounded by, 0 < n .leq. L*m.
%       g         - Chose g=0 for closed loop systems, i.e. E=0..
%       J         - Past horizon used to define instruments. Choosen such
%                   that (A-KD)J \approx 0.
%       n         - Model order, 0 < n .leq. L m.
%
%                                        COPYRIGHT 2004, DDIR
%                                        License belong to: unspecified
%                                        Product id: 10 000
% ------------------------------------------------------------------------

% DDIR, 030104.

