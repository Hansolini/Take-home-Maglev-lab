function [tspan, x_plus_e_, x_minus_e_] = obs_sim(f, x0, input, T, epsilon, P1, P1_inv)

%% Setting up some things:
% Number of states:
states = size(x0, 1);

% Initialising unit vectors
e_ = mat2cell(eye(states),[states], ones(1, states));

% Initialising time vector
t_step = T/100; % Tried with T/1000. It took 4 times longer and the reult was basically the same. At least for lui.
tspan = 0:t_step:T;

%% Scaling the states such that they have approximately the same scale:
% x_ = P1 x
f_mod = @(x_, u) P1*f(P1_inv*x_, u);


%% Simulating
% Positive perturbations:
t_plus_e_ = cell(states, 1);
x_plus_e_ = cell(states, 1);
for n = 1:states
    [t_plus_e_{n}, x_plus_e_{n}] = ode15s(@(t, x) f_mod(x, input), tspan, P1*x0 + epsilon*e_{n});
end

% Negative perturbations:
t_minus_e_ = cell(states, 1);
x_minus_e_ = cell(states, 1);
for n = 1:states
    [t_minus_e_{n}, x_minus_e_{n}] = ode15s(@(t, x) f_mod(x, input), tspan, P1*x0 - epsilon*e_{n});
end



end