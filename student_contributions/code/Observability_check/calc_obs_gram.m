function [obs_gram] = calc_obs_gram(h, input, tspan, x_plus_e_, x_minus_e_, epsilon, P1_inv, P2, measurements_to_include)

%% Setting up some things:
% Number of states and measurements:
sample_state = x_plus_e_{1}(1, :); % Used to find the number of measurements and states.

states = size(sample_state, 2);
measurements = size(h(sample_state.', input), 1);


%% Output scale such that noise covariance is I:
% y_ = P2 y
h_mod = @(x_, u) P2*h(P1_inv*x_, u);



%% Calculate the output trajectories from state trajectories:
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
        for n = measurements_to_include %1:measurements
            integrand = integrand + a(n, :).*b(n, :);
        end
        % integrand = sum(a.*b);

        integral = trapz(tspan, integrand);
        obs_gram(row, column) = (1/(4*epsilon^2))*integral;
    end
end




end