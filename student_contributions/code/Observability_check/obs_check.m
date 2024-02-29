function [lui, lecn, tspan, x_plus_e_, x_minus_e_] = obs_check(f, h, x0, input, T, epsilon, states_to_include, measurements_to_include,  P1, P1_inv, P2)

%% Setting up some things:
% Number of states and measurements:
%states = size(x0, 1);
%measurements = size(h(x0, input), 1);



%% Simulating, creating observability gramian and calculating lui and lecn:
[tspan, x_plus_e_, x_minus_e_] = obs_sim(f, x0, input, T, epsilon, P1, P1_inv);

[obs_gram] = calc_obs_gram(h, input, tspan, x_plus_e_, x_minus_e_, epsilon, P1_inv, P2, measurements_to_include);

[lui, lecn] = lui_and_lecn(obs_gram, states_to_include);

end





%% Comments:

% Done:

% Scale coordinates and outputs

% Simulate this: x+e1, x+e2 ... x+e12, then, x-e1, x-e2 ... x-e12
% and store the output trajectories.

% Initialize a 12x12 zero-matrix.
% Calculate all the elements of the empirical local observability gramian
% using formula (4) from the paper inside a nested for loop.
% Place the elements in the initialized matrix after they have been
% calculated.