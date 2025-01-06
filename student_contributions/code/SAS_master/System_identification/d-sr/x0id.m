function x = x0id(y,u,a,b,d,e,c,L);
% X0ID Computes the initial state for a given model on
% innovations form (Kalman filter) and given data (Y and U.
% SYNTAX
% x1 = x0id(y,u,a,b,d,e,c,L);
% ON INPUT
% Y,U     - Output and input data matrices, respectively.
% a,b,d,e - Deterministic part of model
% c       - The kalman filter gain matrix.
%           Put c=0 for a pure deterministic model.
% L       - The state identification horizon.
% ON OUTPUT
% X1      - The initial state vector.

