%parameters_maggy_V2;
%modelName = 'fast';

%correctionFactorFast = 0.558361865282244;
%paramsFast = params;
%paramsFast.solenoids.r = correctionFactorFast*paramsFast.solenoids.r;

%%
no_input = zeros(4, 1);
states_to_include = [1:5];
measurements_to_include = [1:9];



r_factor = [0.01 0.05 0.1 0.2 0.5 1 1.5 2 3 5 1e1 1e2 1e3 1e4 1e5 1e6 1e7 1e8 1e9 1e10 1e11 1e12 1e13 1e14];
% r_factor = [1e1 1e2 1e3 1e4 1e5 1e6 1e7 1e8 1e9 1e10 1e11 1e12 1e13 1e14];
points = size(r_factor, 2);

%%
for n = 1:points
    paramsFastMovedSensors = paramsFast;
    paramsFastMovedSensors.sensors.x = paramsFast.sensors.x*r_factor(n)*n;
    paramsFastMovedSensors.sensors.y = paramsFast.sensors.y*r_factor(n)*n;
    h = @(x,u) maglevSystemMeasurements(x,u,paramsFastMovedSensors ,modelName);

    h([-1e5;0;zEq;0;0;0;0;0;0;0;0;0], no_input)
end
%%


wait = waitbar(0, sprintf("0 out of %d positions done.", points));

lui_average = [];
lecn_average = [];


for n = 1:points
paramsFastMovedSensors = paramsFast;
paramsFastMovedSensors.sensors.x = paramsFast.sensors.x*r_factor(n)*n;
paramsFastMovedSensors.sensors.y = paramsFast.sensors.y*r_factor(n)*n;
h = @(x,u) maglevSystemMeasurements(x,u,paramsFastMovedSensors ,modelName);

% paramsFastMovedSensors = paramsFast;
% paramsFastMovedSensors.sensors.x = [0, 0, 0.0212];
% paramsFastMovedSensors.sensors.y = [0 0.0212 0];
% h = @(x,u) maglevSystemMeasurements(x,u,paramsFastMovedSensors ,modelName);

lui = [];
lecn = [];
for row_index = 41:50
    for column_index = 52:61
        
        [obs_gram] = calc_obs_gram(h, no_input, tspan, x_plus_e_{row_index, column_index}, x_minus_e_{row_index, column_index}, epsilon, P1_inv, P2, measurements_to_include);
        [lui(row_index, column_index), lecn(row_index, column_index)] = lui_and_lecn(obs_gram, states_to_include);
        
        
    end 
end
lui = lui(41:50, 52:61);
lecn = lecn(41:50, 52:61);
lui_average(n) = mean(lui, "all");
lecn_average(n) = mean(lecn, "all");

%%

% Update waitbar:
points_done = n;
waitbar(n/points, wait,  sprintf("%d out of %d positions done.", points_done, points));
end

pause(250e-3);
delete(wait);