function [a,d,cf,f,x0,sn]=dsr_s(y,L,k,bmet,n)
% DSR_S Stochastic system identification and Realization
%      [A,B,D,E,CF,F,x0]=dsr_s(Y,L)
%      [A,B,D,E,CF,F,x0]=dsr_s(Y,L,J)
%      [A,B,D,E,CF,F,x0]=dsr_s(Y,L,J,M,n)
%      PURPOSE:
%      Estimate the system order (n) and the matrices (A,D,CF,F)
%      in the following discrete time stochastic dynamic model 
%      on innovations form
%
%      x_{t+1} = A x_t + CF ep_t,  x_{t=0}=x0, E(ep_t ep_t^T)=I,
%      y_t     = D x_t + F ep_t,
%
%      which is equivalent with the standard innovations (prediction) form
%
%      x_{t+1} = A x_t + C e_t,  x_{t=0}=x0,
%      y_t     = D x_t + e_t,
%
%      where C     = CF*inv(F),       (Kalman gain, K =inv(A)*C and C=A*K),
%      Delta = E(e_t e_t^T) = F*F',   (Innovations noise covariance matrix).
%
%      ON INPUT:
%      Y         - Output time series matrix of size (N x m) 
%                  where N is the number of observations and
%                  m is the number of output variables.
%      L         - Number of block rows in extended observability matrix.
%                  Choose L .geq. 1. This means that one can estimate
%                  system order (n) bounded by, 0 < n .leq. L*m.
%      OPTIONAL INPUT PARAMETERS:
%      J         - Past horizon used to define instruments. Default, J=L.
%      M         - Default M=1. See tutorial.
%      n         - Optional specification of model order, 0 < n .leq. L m.
%      ON OUTPUT:
%      A,D       - Model system matrices.
%      CF, F     - C=CF*inv(F) is the Kalman filter gain matrix.
%      F         - Delta = F*F' is the innovation noise covariance matrix.
%      x0        - Initial values for the state vector, x_t, i.e. state at t=0.
%
%                                       COPYRIGHT 1996, 1999, DDIR
%                                       License belong to:
%                                       Product id: 10 000
%------------------------------------------------------------------------

% DATE: 1. november 1996
% Notes: 
% 4. Algorithm: Di Ruscio (1996), A Method for ...,
%               In "Computer Aided Time series Modeling",
%               Ed: M. Aoki, Springer Verlag.
% 5. J must satisfy J > 0 in order to define the instruments.
%------------------------------------------------------------------------

% low level functions: seye, sobsv, dread
