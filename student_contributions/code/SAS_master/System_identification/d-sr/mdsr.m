function [a,b,d,e,cf,f,x0]=mdsr(y,u,L,g,J,bmet,n,Ni)
% [a,b,d,e,cf,f,x0]=mdsr(Y,U,L,g,J,M,n,NN)
% MDSR Multiple DSR. Deterministic and Stochastic system identification 
%      and Realization from possible multiple time series.
%      [A,B,D,E,CF,F,x0]=mdsr(Y,U,L)
%      [A,B,D,E,CF,F,x0]=mdsr(Y,U,L,g)
%      [A,B,D,E,CF,F,x0]=mdsr(Y,U,L,g,J,M,n,NN)
%      PURPOSE:
%      Estimate the system order (n) and the matrices (A,B,D,E,CF,F)
%      in the following discrete time combined deterministic and 
%      stochastic dynamic model on innovations form
%      x_{t+1} = A x_t + B u_t + C e_t,    x_{t=0}=x0,
%      y_t     = D x_t + E u_t + e_t,
%      where C     = CF*inv(F),       (Kalman gain matrix, (C=inv(A) K),
%      Delta = E(e_t e_t^T) = F*F',   (Innovations noise covariance matrix).
%
%      MULTIPLE TIME SERIES:
%      Assume that the following multiple time series are given.
%      Y_i of size (Ni x ny) and U_i of size (Ni x nu) for all i=1,...,Ne.
%      Specify parameters and data-matrices as follows:
%      NN=[N1 N2 ... Ne]
%      Y =[Y1;Y2;...;Y_Ne] an (sum(NN) x ny) data matrix.
%      U =[Y1;Y2;...;Y_Ne] an (sum(NN) x nu) data matrix.
%      Example of call: Given N1=100 samples in (Y1,U1) and N2=200 in (Y2,U2),
%      [a,b,d,e,cf,f,x0]=mdsr([Y1;Y2],[U1;U2],L,1,L,1,[],[100 200]);
%
%      ON INPUT:
%      Y         - Output time series matrix of size (N x m) 
%                  where N is the number of observations and
%                  m is the number of output variables.
%      U         - Input time series matrix of size (N x r)
%                  where r is the number of input variables.
%      L         - Number of block rows in extended observability matrix.
%                  Choose L .geq. 1. This means that one can estimate
%                  system order (n) bounded by, 0 < n .leq. L*m.
%      OPTIONAL INPUT PARAMETERS:
%      g         - g=1 default, g=0 force E to be the zero matrix.
%      J         - Past horizon used to define instruments. Default, J=L.
%      M         - Default M=1. See tutorial.
%      n         - Optional specification of model order, 0 < n .leq. L m.
%                  n=[] --> order identified.
%      NN        - In case of multiple time series. NN=[N1...Ni...Ne].
%      ON OUTPUT:
%      A,B,D,E   - Model system matrices.
%      CF, F     - C=CF*inv(F) is the Kalman filter gain matrix.
%      F         - Delta = F*F' is the innovation noise covariance matrix.
%      x0        - Initial values for the state vector, x_t, i.e. state at t=0.
%                  x0 is a matrix in case of multiple time series, i.e.
%                  when NN is specified as NN=[N1...Ne].
%------------------------------------------------------------------------

% low level functions: seye, sobsv, simpr, besolv, dread

% Written for project PR8-41464.01 by David Di Ruscio. 12/3-98.
% Based on an idea by Terje Karstang of making a 3-dimensional DSR algorithm.

