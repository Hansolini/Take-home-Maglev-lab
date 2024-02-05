clear all; close all; clc;

state_standard_size = 1;
epsilon = state_standard_size*1e-3;

% Todo:




% Scale coordinates and outputs

% Simulate this: x+e1, x+e2 ... x+e12, then, x-e1, x-e2 ... x-e12
% and store the output trajectories.

% Initialize a 12x12 zero-matrix.
% Calculate all the elements of the empirical local observability gramian
% using formula (4) from the paper inside a nested for loop.
% Place the elements in the initialized matrix after they have been
% calculated.