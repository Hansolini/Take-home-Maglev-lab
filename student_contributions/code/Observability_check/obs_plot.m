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

T = 10e-3;
epsilon = 1e-5;

grid_size = 15e-3;
grid_resolution = 3;

if round(mod(grid_resolution, 2)) == 0
    disp("Please use an odd number such that the equilibrium is included in the grid.")
    return
end


x = linspace(-grid_size, grid_size, grid_resolution);
y = linspace(grid_size, -grid_size, grid_resolution);

%lui
%lecn
lui = zeros(grid_resolution);
lecn = zeros(grid_resolution);
for row_index = 1:size(y, 2)
    for column_index = 1:size(x, 2)
        x0 = [x(column_index),y(row_index),zEq(1),zeros(1,9)]';
        [lui(row_index, column_index), lecn(row_index, column_index)] = obs_check(f, h, x0, no_input, T, epsilon);
        %a(k, l) = k;
        %b(k, l) = l;
    end
end

%% Plot


