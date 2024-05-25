clear all; close all; clc;
%{
%% Sensor placement computation
% This file computes the optimal sensor placement of Maggy V3.0 - in terms 
of state observability - using the observability_check contribution of 
S. Graffer. Maggy V3.0 allows for changing the placement of the permanent
magnets and the solenoids. 

This file computes the the optimal placement of the sensors given 
different configurations of the permament magnets. The output is thus a 
graph showing the "optimality curve" for the different magnet 
configurations, with the optimums highlighted.
%}

%% Initializing parameters
% Load parameters
parameters_maggy_V2;

% Model selection and parameter correction
modelName = 'fast';
correctionFactorFast = 0.558361865282244;
paramsFast = params;
paramsFast.solenoids.r = correctionFactorFast*paramsFast.solenoids.r;

%% Define magnet configurations
PERMANENT_MAGNET_CONFIG = {};

r_permanent = 0.02 + (0:0.0016:0.0016*3);
for i = 1:length(r_permanent)
    permanent = paramsFast.permanent;
    permanent.x = r_permanent(i)*[1 -1 1 -1];
    permanent.y = r_permanent(i)*[1 1 -1 -1];
    PERMANENT_MAGNET_CONFIG{end+1} = permanent;
end

%% Initialize observability check parameters
% Setting up scaling factors for the states:
a = 0.5e-3;
b = 2*(pi/180);
c = a*100;
d = b*100;

% x_ = P1 x
P1_inv = diag([a a a b b b c c c d d d]);
P1 = inv(P1_inv);

% And for the measurements:
measurement_noise_sigma = diag(kron(ones(1, 3), 1e-3*[0.0084 0.0085 0.0124].^0.5));

% y_ = P2 y
P2_inv = measurement_noise_sigma;
P2 = inv(P2_inv);

%% Initialize simulation/observation check parameters
% Choose simulation time and epsilon used for the observability gramian
T = 10e-3;
epsilon = 1e-3;
states_to_include = 1:5;
measurements_to_include = 1:9;

% Define simulation grid
width_grid = 0.01;
n_grid = 9;

if round(mod(n_grid, 2)) == 0
    disp("Please use an odd number such that the equilibrium is included in the grid!")
    return
end

[X,Y] = meshgrid(linspace(-width_grid, width_grid, n_grid));

% Define radial vector for sensor placement
r_sensors = linspace(0,max(r_permanent)*10,50);

%% Initialize figure
figure(1);
clf; 
title('Unobservability for different sensor positions')

subplot(2,1,1);
grid on; hold on; box on;
xlabel('Radial position of sensors')
ylabel('LUI')

subplot(2,1,2);
grid on; hold on; box on;
xlabel('Radial position of sensors')
ylabel('LECN')

%% Computing observability graph for each configuration
for i = 1:length(PERMANENT_MAGNET_CONFIG)
    clc;
    fprintf('Configuration %.f\n', i);

    % Filename
    filename = sprintf('config_%.f.mat', i);
    
    %% Step 1: Run simulation
    fprintf('Simulation\n')

    if isfile(sprintf('data/simulation_%s',filename))
        load(sprintf('data/simulation_%s',filename));
    else % Run simulation if data is not there
        % Modify parameters
        paramsFast.permanent = PERMANENT_MAGNET_CONFIG{i};
    
        % Initialize system functions
        f = @(x,u) maglevSystemDynamics(x,u,paramsFast,modelName);
        h = @(x,u) maglevSystemMeasurements(x,u,paramsFast,modelName);
        
        % Linearize around system equilibria
        [zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(paramsFast,'fast');

        % Run simulation for each grid point
        x_plus_e_ = cell(size(X));
        x_minus_e_ = cell(size(X));
        for j = 1:size(X,1)
            for k = 1:size(X,2)
                % Initial point 
                x0 = [X(j,k), Y(j,k), zEq(1), 0, 0, zeros(1,7)]';
    
                % Run simulation
                [tspan, x_plus_e_{j,k}, x_minus_e_{j,k}] = obs_sim(f, x0, zeros(4,1), T, epsilon, P1, P1_inv);
            end
        end

        % Save simulation results
        save(sprintf('data/simulation_%s',filename),'x_plus_e_','x_minus_e_','tspan','paramsFast','params','zEq','f','h')
    end

    %% Step 2: Compute observability check
    fprintf('Observability check\n')

    if isfile(sprintf('data/observability_%s',filename))
        load(sprintf('data/observability_%s',filename));
    else % Run observability check if data is not there
        lui  = zeros(size(r_sensors));
        lecn = zeros(size(r_sensors));
        for j = 1:length(r_sensors)
            fprintf('Radius nr. %.f\n', j)

            % Modify sensor positions
            sensors = paramsFast.sensors;
            sensors.x = r_sensors(j)*[0 1  1];
            sensors.y = r_sensors(j)*[0 1 -1];
            paramsFast.sensors = sensors;

            h = @(x,u) maglevSystemMeasurements(x,u,paramsFast,modelName);

            % Run observability check (for a subset of points)
            LUI  = zeros(size(X));
            LECN = zeros(size(X));
            for k = 1:size(X,1)
                for l = 1:size(X,2)
                    if X(k,l) == 0 || Y(k,l) == 0 % Skip uninteresting points
                        continue;
                    end
                    [obs_gram] = calc_obs_gram(h, zeros(4,1), tspan, x_plus_e_{k,l}, x_minus_e_{k,l}, epsilon, P1_inv, P2, measurements_to_include);
                    [LUI(k,l), LECN(k,l)] = lui_and_lecn(obs_gram, states_to_include);
                end
            end

            % Compute average lui and lecn
            lui_average(j)  = mean(LUI,'all');
            lecn_average(j) = mean(LECN,'all');
        end

        % Save observability results
        save(sprintf('data/observability_%s',filename))
    end

    %% Step 3: Compute optimum and add to plot
    [lui_average_min, I_lui] = min(lui_average);
    [lecn_average_min, I_lecn] = min(lecn_average);

    figure(1);
    subplot(2,1,1);
    plot(r_sensors, lui_average, 'linewidth', 2, 'DisplayName',sprintf('Configuration %.f | r^* = %.f', i, r_sensors(I_lui)));
    plot(r_sensors(I_lui), lui_average_min, 'rx', 'handlevisibility', 'off');
    xlim([min(r_sensors), max(r_sensors)])

    subplot(2,1,2);
    plot(r_sensors, lecn_average, 'linewidth', 2, 'DisplayName',sprintf('Configuration %.f | r^* = %.f', i, r_sensors(I_lecn)));
    plot(r_sensors(I_lecn), lecn_average_min, 'rx', 'handlevisibility', 'off');
    xlim([min(r_sensors), max(r_sensors)])

    drawnow;
end