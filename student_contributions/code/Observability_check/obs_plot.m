%% Initialising model:
clear all; close all; clc;

% Preparing the simulator:
parameters_maggy_V2;
modelName = 'fast';

correctionFactorFast = computeSolenoidRadiusCorrectionFactor(params,'fast');
% correctionFactorAccurate = computeSolenoidRadiusCorrectionFactor(params,'accurate');

paramsFast = params;
paramsFast.solenoids.r = correctionFactorFast*paramsFast.solenoids.r;

% paramsAccurate = params;
% paramsAccurate.solenoids.r = correctionFactorAccurate*paramsAccurate.solenoids.r;

[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(paramsFast,'fast');

f = @(x,u) maglevSystemDynamics(x,u,paramsFast,modelName);
h = @(x,u) maglevSystemMeasurements(x,u,paramsFast,modelName);

xLp = [0,0,zEq(1),zeros(1,9)]'; % Linearizing around the equilibria
uLp = zeros(length(params.solenoids.r),1);



%%
clc

no_input = zeros(4, 1);

% Choose simulation time and epsilon used 
% to calculate the observability gramian:
T = 10e-3;
epsilon = 1e-4;
states_to_include = [1:5];%[1:5 7:11];

% Define grid size and the number of points in a row or column in the grid:
grid_size = 30e-3;
grid_points = 3;

% Make sure that the grid includes the equilibrium:
if round(mod(grid_points, 2)) == 0
    disp("Please use an odd number such that the equilibrium is included in the grid!")
    return
end

% Create grid:
x = linspace(-grid_size, grid_size, grid_points);
y = linspace(grid_size, -grid_size, grid_points);

% Run observability check for every point in the grid:
lui = zeros(grid_points);
lecn = zeros(grid_points);
timestamps = zeros(grid_points);
wait = waitbar(0, sprintf("0 out of %d points done.", grid_points^2));
for row_index = 1:size(y, 2)
    for column_index = 1:size(x, 2)
        tic;
        x0 = [x(column_index),y(row_index),zEq(1),zeros(1,9)]';
        [lui(row_index, column_index), lecn(row_index, column_index)] = obs_check(f, h, x0, no_input, T, epsilon, states_to_include);
        
        % Save timestamp:
        timestamps(row_index, column_index) = toc;

        % Compute final time
        seconds_left = mean(timestamps(1:row_index,1:column_index),'all')*sum(timestamps(:) == 0);
        current_time = datetime('now');
        final_time = current_time + seconds(seconds_left);

        % Update waitbar:
        points_done = grid_points*(row_index - 1) + column_index;
        waitbar(points_done/grid_points^2, wait,  sprintf("%d out of %d points done.\n Final time: %s", points_done, grid_points^2, datestr(final_time)));
    end
    %save('obs_plot.mat')
end
pause(250e-3);
close(wait);

