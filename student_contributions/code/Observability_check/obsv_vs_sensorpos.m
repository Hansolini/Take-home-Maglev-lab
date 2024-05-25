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

% Use this when only testing for r_factor:
%r_factor = [0 logspace(-1, 1, 100)];

% Use this when testing both angle and r_factor:
r_factor = kron(ones(1, 11), [0 logspace(-1, 1, 20)]);
theta = kron(linspace(0, (pi/2), 11), ones(1, 21));

% Outdated:
%r_factor = [0.01 0.05 0.1 0.2 0.5 1 1.5 2 3 5];
%r_factor = [1];


points = size(r_factor, 2);


% Not in use:
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
% Use these lines when only testing for r_factor:
% paramsFastMovedSensors = paramsFast;
% paramsFastMovedSensors.sensors.x = paramsFast.sensors.x*r_factor(n);
% paramsFastMovedSensors.sensors.y = paramsFast.sensors.y*r_factor(n);
% h = @(x,u) maglevSystemMeasurements(x,u,paramsFastMovedSensors ,modelName);

% Use these lines instead when testing both angle and r_factor:
angle = theta(n);
paramsFastMovedSensors = paramsFast;
paramsFastMovedSensors.sensors.x = [0 0.0212*cos(angle)      0.0212*sin(angle)]*r_factor(n);
paramsFastMovedSensors.sensors.y = [0 0.0212*sin(angle)      0.0212*-cos(angle)]*r_factor(n);
h = @(x,u) maglevSystemMeasurements(x,u,paramsFastMovedSensors ,modelName);

lui = [];
lecn = [];

row_range = [47:50 52:55];%41:50;
column_range = [47:50 52:55];%52:61;

for row_index = row_range
    for column_index = column_range
        
        [obs_gram] = calc_obs_gram(h, no_input, tspan, x_plus_e_{row_index, column_index}, x_minus_e_{row_index, column_index}, epsilon, P1_inv, P2, measurements_to_include);
        [lui(row_index, column_index), lecn(row_index, column_index)] = lui_and_lecn(obs_gram, states_to_include);
        
        
    end 
end
lui = lui(row_range, column_range);
lecn = lecn(row_range, column_range);
lui_average(n) = mean(lui, "all");
lecn_average(n) = mean(lecn, "all");

%%

% Update waitbar:
points_done = n;
waitbar(n/points, wait,  sprintf("%d out of %d positions done.", points_done, points));
end

pause(250e-3);
delete(wait);

%clear x_minus_e_ x_plus_e_
%save("radial_sensor_placement_20_5.mat")

%%
% figure(1);
% 
% clf;
% 
% subplot(2,1,1);
% 
% grid on; hold on; box on;
% 
% ylabel('lui')
% plot(r_factor,lui_average,'b', 'LineWidth' ,2)
% %set(gca, 'xscale’,‘log')
% 
% xlabel('Relative position')
% 
% ylim([14, 40])
% 
% subplot(2,1,2);
% 
% grid on; hold on; box on;
% 
% plot(r_factor, lecn_average, 'b', 'LineWidth' ,2)
% ylabel('lecn' )
% 
% xlabel( 'Relative position')
% 
% ylim([3,7])
% 
% %set(gca, 'xscale’,‘log'’)