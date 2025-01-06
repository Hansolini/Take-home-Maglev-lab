function [a,b,d,e,f,x0]=dsr_oe(y,u,L,g,k,bmet,n)
% DSR_OE Deterministic system identification and Realization
%      [A,B,D,E,F,x0]=dsr_oe(Y,U,L)
%      [A,B,D,E,F,x0]=dsr_oe(Y,U,L,g)
%      PURPOSE:
%      Estimate the system order (n) and the matrices (A,B,D,E,F)
%      in the following discrete time output error model.
%
%      x_{t+1} = A x_t + B u_t,          x_{t=0}=x0
%      y_t     = D x_t + E u_t + e_t
%
%      where C     = CF*inv(F)       Kalman gain matrix, (C=inv(A) K))
%      Delta = E(e_t e_t^T) = F*F'   Innovations noise covariance matrix
%
%      ON INPUT:
%      Y         - Output time series matrix of size (N x m) 
%                  where N is the number of observations and
%                  m is the number of output variables.
%      U         - Input time series matrix of size (N x r)
%                  where r is the number of input variables.
%      L         - Number of block rows in extended observability matrix.
%                  Choose L .geq. 1. This means that one can estimate
%                  system order (n) bounded by, n .leq. L*m.
%      g         - g=1 default, g=0 force E to be the zero matrix.
%      ON OUTPUT:
%      A,B,D,E   - model system matrices
%      F         - Delta = F*F', The innovation noise covariance matrix
%      x0        - initial values for the state vector, x_t, i.e. state at t=0.
%
%                                       COPYRIGHT 1996, FANTOFT PROCESS
%                                       License belong to Terje Karstang
%                                       Product id: 10 0000
%------------------------------------------------------------------------

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
%------------------------------------------------------------------------

% low level functions: seye, sobsv, simpr, besolv, dread
