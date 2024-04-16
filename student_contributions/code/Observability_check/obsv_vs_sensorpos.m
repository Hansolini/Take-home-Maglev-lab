%parameters_maggy_V2;
%modelName = 'fast';

%correctionFactorFast = 0.558361865282244;
%paramsFast = params;
%paramsFast.solenoids.r = correctionFactorFast*paramsFast.solenoids.r;

%%
% loading the simulation
%load("obs_plot.mat")


no_input = zeros(4, 1);
states_to_include = [1:5];
measurements_to_include = [1:9];


r_factor = [0 logspace(-1, 1, 100)];
%r_factor = [0.01 0.05 0.1 0.2 0.5 1 1.5 2 3 5];
%r_factor = [1];
points = size(r_factor, 2);


%% Fixing the noise bug
% And for the measurements:
measurement_noise_sigma = diag(kron(ones(1, 3), 1e-3*[0.0084 0.0085 0.0124].^0.5));

% y_ = P2 y
P2_inv = measurement_noise_sigma;
P2 = inv(P2_inv);
%%

%%
% for n = 1:points
%     paramsFastMovedSensors = paramsFast;
%     paramsFastMovedSensors.sensors.x = paramsFast.sensors.x*r_factor(n);
%     paramsFastMovedSensors.sensors.y = paramsFast.sensors.y*r_factor(n);
%     h = @(x,u) maglevSystemMeasurements(x,u,paramsFastMovedSensors ,modelName);
% 
%     h([-1e5;0;zEq;0;0;0;0;0;0;0;0;0], no_input)
% end
%%


wait = waitbar(0, sprintf("0 out of %d positions done.", points));

lui_average = [];
lecn_average = [];


for n = 1:points
paramsFastMovedSensors = paramsFast;
paramsFastMovedSensors.sensors.x = paramsFast.sensors.x*r_factor(n);
paramsFastMovedSensors.sensors.y = paramsFast.sensors.y*r_factor(n);
h = @(x,u) maglevSystemMeasurements(x,u,paramsFastMovedSensors ,modelName);

% paramsFastMovedSensors = paramsFast;
% paramsFastMovedSensors.sensors.x = [0, 0, 0.0212]*0.35;
% paramsFastMovedSensors.sensors.y = [0 0.0212 0]*0.35;
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

clear x_minus_e_ x_plus_e_
save("radial_sensor_placement_16_4.mat")