function yp_M=dsr_prd(A,B,D,E,C,F,y,u,x0,M)
% DSR_PRD Computes the M-step ahead prediction of y_t.
% SYNTAX
% Yp_M = dsr_prd(A,B,D,E,CF,F,Y,U,x0,M)
% DESCRIPTION
% Old outputs up to time t-M and all relevant inputs 
% are used to predict the output at time t. 
% ON INPUT
% A B D E CF F x0 - Innovations form model matrices and initial state, x0,
%                   as computed by DSR, i.e., the outputs from DSR. 
% Y, U            - The output and input data matrices, respectively, i.e., 
%                   Y is an (N x m) data matrix and U is an (N x r) data matrix,
%                   where N is the number of observations, m is the number of outputs
%                   and r is the number of input variables.
% M               - The prediction horizon, i.e., an integer M>=1.
% ON OUTPUT
% Yp_M            - Matrix with the M-step ahead predictions, i.e. a (N x m) matrix.
%
% FUNCTIONS CALLED: none
% Copyright 2000, Dr. ing. David Di Ruscio. 

% Written 3/2-00

