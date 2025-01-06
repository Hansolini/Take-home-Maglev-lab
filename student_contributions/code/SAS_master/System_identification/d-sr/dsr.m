function [a,b,d,e,cf,f,x0,sn]=dsr(y,u,L,g,k,bmet,n)
% DSR  Deterministic and Stochastic system identification and Realization
%      [A,B,D,E,CF,F,x0]=dsr(Y,U,L)
%      [A,B,D,E,CF,F,x0]=dsr(Y,U,L,g)
%      [A,B,D,E,CF,F,x0]=dsr(Y,U,L,g,J,M,n)
%      PURPOSE:
%      Estimate the system order (n) and the matrices (A,B,D,E,CF,F)
%      in the following discrete time combined deterministic and 
%      stochastic dynamic model on innovations form
%
%      x_{t+1} = A x_t + B u_t + C e_t,    x_{t=0}=x0,
%      y_t     = D x_t + E u_t + e_t,
%
%      where C     = CF*inv(F),       (Kalman gain, K =inv(A)*C and C=A*K),
%      Delta = E(e_t e_t^T) = F*F',   (Innovations noise covariance matrix).
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
%      ON OUTPUT:
%      A,B,D,E   - Model system matrices.
%      CF, F     - C=CF*inv(F) is the Kalman filter gain matrix.
%      F         - Delta = F*F' is the innovation noise covariance matrix.
%      x0        - Initial values for the state vector, x_t, i.e. state at t=0.
%
%                                       COPYRIGHT 1996, 1999, DDIR
%                                       License belong to:
%                                       Product id: 10 000
%------------------------------------------------------------------------

% DATE: 9, january 2004. Better method for setting the system order.
%       1. november 1996
% Notes: 
% 1. Choose L as close to the observability index (n-d+1) as possible
%    L >= n-d+1, n >=d whenever the input is poor with frequencies
%    where d=rank(y_t) usually equal to m.
%    (however, this is not necessary)
% 2. In case that E=0, then chose parameter g=0.
% 3. k, bmet, n. Optional parameters for advanced use.
%    (k=L default, bmet=1
% 4. Algorithm: Di Ruscio (1996), A Method for ...,
%               In "Computer Aided Time series Modeling",
%               Ed: M. Aoki, Springer Verlag.
% 5. J must satisfy J > 0 in order to define the instruments.
%------------------------------------------------------------------------
