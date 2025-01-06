% D-SR Toolbox for Matlab
% Version 1.0
% Copyright 2000, Dr. ing. David Di Ruscio
% Licence belong to:
% Product id: 10 003
% $Revision: 1.1 $ $Date: February 3, 2000$
%
% 1. DSR           % Dynamic modeling and identification.
% dsr.m            % Main function. Compute model from data (Y,U).
%                  % Use dsr.m when both inputs, U, and outputs, Y
%                  % are available.
% dsr_e.m          % Main function for closed loop subspace system
%                  % identification.
% dsr_s.m          % Use dsr_s.m when only outputs, Y, are available.
%
% mdsr.m           % Dynamic modeling of multiple series (Y1,U1), ...,(Ym,Um)
%
% dsr_oe.m         % Modeling of deterministic systems. efficient if U is
%                  % a poorly exiting input signal.
%
% 2. DSRSIM        % Simulation.
% dsrsim.m         % Simulation of discrete-time linear systems.
%
% 3. DSROPT           % Optimal prediction and simulation.
% dsropt.m         % Compute optimal predictions from the model and the data.
%
% 4. DSR_PRD       % M-step ahead predictions.
% dsr_prd.m        % Computes M-step ahead predictions from the state space model 
%                  % and the relevant data.
% 5. Low level DSR functions
% besolv.m
% dread.m
% seye.m
% simpr.m
% sobsv.m
%
% 6. Utility function
% ss2cf.m          % Transform a MIMO state space model to observable
%                  % canonical form.
% x0id.m           % Compute the initial state x0 for a given model and data.