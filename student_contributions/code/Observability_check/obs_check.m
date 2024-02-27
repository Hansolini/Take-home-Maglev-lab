function [lui, lecn, tspan, x_plus_e_, x_minus_e_] = obs_check(f, h, x0, input, T, epsilon, states_to_include)

%% Setting up some things:
% Number of states and measurements:
states = size(x0, 1);
measurements = size(h(x0, input), 1);

% Initialising unit vectors
e_ = mat2cell(eye(states),[states], ones(1, states));

% Initialising time vector
t_step = T/100; % Tried with T/1000. It took 4 times longer and the reult was basically the same. At least for lui.
tspan = 0:t_step:T;



%% Coordinate transform:
% Scale the states such that they have approximately the same scale:
a = 20e-3/10;
b = (pi/2)/10;

c = a*100;%100e-3/10;
d = b*100;%(3*2*pi)/10;

% x_ = P1 x
P1_inv = diag([a a a b b b c c c d d d]);
P1 = inv(P1_inv);

f_mod = @(x_, u) P1*f(P1_inv*x_, u);


% Output scale such that noise covariance is I:
measurement_noise_covariance = 1e-3*diag(kron(ones(1, 3), [0.0084 0.0085 0.0124]));

% y_ = P2 y
P2_inv = measurement_noise_covariance.^0.5;
P2 = inv(P2_inv);

h_mod = @(x_, u) P2*h(P1_inv*x_, u);


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


%% Calculate the output trajectories:
% Preparing function that takes in a simulation result matrix and 
% calculates the outputs for each point in time:
sample_points = size(tspan, 2);
no_input_bulk = kron(ones(1, sample_points), input);

h_mod_bulk = @(x_bulk, u_bulk) cell2mat(cellfun(h_mod, num2cell(x_bulk, 1), num2cell(u_bulk, 1), 'UniformOutput', false));


% Positive perturbations:
y_plus = cell(states, 1);
for n = 1:states
    y_plus{n} = h_mod_bulk(x_plus_e_{n}.', no_input_bulk);
end

% Negative perturbations:
y_minus = cell(states, 1);
for n = 1:states
    y_minus{n} = h_mod_bulk(x_minus_e_{n}.', no_input_bulk);
end


%%
% The empirical local observability gramian is initialized here:
obs_gram = zeros(states);

for row = 1:states
    for column = 1:states
        %integrand = (y_plus{row} - y_minus{row}).'*(y_plus{column} - y_minus{column});
        a = y_plus{row} - y_minus{row};
        b = y_plus{column} - y_minus{column};
        
        integrand = zeros(size(a(1, :)));
        for n = 1:measurements
            integrand = integrand + a(n, :).*b(n, :);
        end

        integral = trapz(tspan, integrand);
        obs_gram(row, column) = (1/(4*epsilon^2))*integral;
    end
end

obs_gram_red = obs_gram(states_to_include, states_to_include);

%obs_gram_eig = eig(obs_gram);
obs_gram_red_eig = eig(obs_gram_red);


%%  Check the observability
% Local singular values
lsv = sqrt(obs_gram_red_eig);
% Local unobservability index:
lui = 1/min(lsv);
% Local estimation condition number:
lecn = sqrt(max(obs_gram_red_eig)/min(obs_gram_red_eig));




%% 
% Done:



% Scale coordinates and outputs

% Simulate this: x+e1, x+e2 ... x+e12, then, x-e1, x-e2 ... x-e12
% and store the output trajectories.

% Initialize a 12x12 zero-matrix.
% Calculate all the elements of the empirical local observability gramian
% using formula (4) from the paper inside a nested for loop.
% Place the elements in the initialized matrix after they have been
% calculated.

%Todo:


end
