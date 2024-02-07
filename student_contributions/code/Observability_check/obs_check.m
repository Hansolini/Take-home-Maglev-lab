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

%% Setting up some things:

% addpath(genpath('../../../simulation'));

e1 = [1;0;0;0;0;0;0;0;0;0;0;0];
e2 = [0;1;0;0;0;0;0;0;0;0;0;0];
e3 = [0;0;1;0;0;0;0;0;0;0;0;0];
e4 = [0;0;0;1;0;0;0;0;0;0;0;0];
e5 = [0;0;0;0;1;0;0;0;0;0;0;0];
e6 = [0;0;0;0;0;1;0;0;0;0;0;0];
e7 = [0;0;0;0;0;0;1;0;0;0;0;0];
e8 = [0;0;0;0;0;0;0;1;0;0;0;0];
e9 = [0;0;0;0;0;0;0;0;1;0;0;0];
e10 = [0;0;0;0;0;0;0;0;0;1;0;0];
e11 = [0;0;0;0;0;0;0;0;0;0;1;0];
e12 = [0;0;0;0;0;0;0;0;0;0;0;1];

% Number of states:
states = 12;

state_standard_size = 1;
epsilon = state_standard_size*1e-6;

T = 20e-3;
t_step = 1e-3;
tspan = 0:t_step:T;

no_input = zeros(4, 1);

%x_eq = zeros(states, 1);        % !!!!!!!! Husk å oppdatere denne



%% Coordinate transform:
% Scale the states such that they have approximately the same scale:
a = 15e-3/10;
b = 100e-3/10;

c = (pi/2)/10;
d = (3*2*pi)/10;

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

[t_plus_e1, x_plus_e1] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp + epsilon*e1);
[t_plus_e2, x_plus_e2] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp + epsilon*e2);
[t_plus_e3, x_plus_e3] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp + epsilon*e3);
[t_plus_e4, x_plus_e4] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp + epsilon*e4);
[t_plus_e5, x_plus_e5] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp + epsilon*e5);
[t_plus_e6, x_plus_e6] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp + epsilon*e6);
[t_plus_e7, x_plus_e7] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp + epsilon*e7);
[t_plus_e8, x_plus_e8] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp + epsilon*e8);
[t_plus_e9, x_plus_e9] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp + epsilon*e9);
[t_plus_e10, x_plus_e10] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp + epsilon*e10);
[t_plus_e11, x_plus_e11] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp + epsilon*e11);
[t_plus_e12, x_plus_e12] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp + epsilon*e12);

[t_minus_e1, x_minus_e1] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp - epsilon*e1);
[t_minus_e2, x_minus_e2] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp - epsilon*e2);
[t_minus_e3, x_minus_e3] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp - epsilon*e3);
[t_minus_e4, x_minus_e4] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp - epsilon*e4);
[t_minus_e5, x_minus_e5] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp - epsilon*e5);
[t_minus_e6, x_minus_e6] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp - epsilon*e6);
[t_minus_e7, x_minus_e7] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp - epsilon*e7);
[t_minus_e8, x_minus_e8] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp - epsilon*e8);
[t_minus_e9, x_minus_e9] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp - epsilon*e9);
[t_minus_e10, x_minus_e10] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp - epsilon*e10);
[t_minus_e11, x_minus_e11] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp - epsilon*e11);
[t_minus_e12, x_minus_e12] = ode15s(@(t, x) f_mod(x, no_input), tspan, P1*xLp - epsilon*e12);

% Calculate the output trajectories:
%h = @(x, u) [eye(3) zeros(3, 9)]*x;
no_input_bulk = kron(ones(1, 21), no_input);

h_mod_bulk = @(x_bulk, u_bulk) cell2mat(cellfun(h_mod, num2cell(x_bulk, 1), num2cell(u_bulk, 1), 'UniformOutput', false));


y_plus_e1 = h_mod_bulk(x_plus_e1.', no_input_bulk);
y_plus_e2 = h_mod_bulk(x_plus_e2.', no_input_bulk);
y_plus_e3 = h_mod_bulk(x_plus_e3.', no_input_bulk);
y_plus_e4 = h_mod_bulk(x_plus_e4.', no_input_bulk);
y_plus_e5 = h_mod_bulk(x_plus_e5.', no_input_bulk);
y_plus_e6 = h_mod_bulk(x_plus_e6.', no_input_bulk);
y_plus_e7 = h_mod_bulk(x_plus_e7.', no_input_bulk);
y_plus_e8 = h_mod_bulk(x_plus_e8.', no_input_bulk);
y_plus_e9 = h_mod_bulk(x_plus_e9.', no_input_bulk);
y_plus_e10 = h_mod_bulk(x_plus_e10.', no_input_bulk);
y_plus_e11 = h_mod_bulk(x_plus_e11.', no_input_bulk);
y_plus_e12 = h_mod_bulk(x_plus_e12.', no_input_bulk);

y_minus_e1 = h_mod_bulk(x_minus_e1.', no_input_bulk);
y_minus_e2 = h_mod_bulk(x_minus_e2.', no_input_bulk);
y_minus_e3 = h_mod_bulk(x_minus_e3.', no_input_bulk);
y_minus_e4 = h_mod_bulk(x_minus_e4.', no_input_bulk);
y_minus_e5 = h_mod_bulk(x_minus_e5.', no_input_bulk);
y_minus_e6 = h_mod_bulk(x_minus_e6.', no_input_bulk);
y_minus_e7 = h_mod_bulk(x_minus_e7.', no_input_bulk);
y_minus_e8 = h_mod_bulk(x_minus_e8.', no_input_bulk);
y_minus_e9 = h_mod_bulk(x_minus_e9.', no_input_bulk);
y_minus_e10 = h_mod_bulk(x_minus_e10.', no_input_bulk);
y_minus_e11 = h_mod_bulk(x_minus_e11.', no_input_bulk);
y_minus_e12 = h_mod_bulk(x_minus_e12.', no_input_bulk);

y_plus{1} = y_plus_e1;
y_plus{2} = y_plus_e2;
y_plus{3} = y_plus_e3;
y_plus{4} = y_plus_e4;
y_plus{5} = y_plus_e5;
y_plus{6} = y_plus_e6;
y_plus{7} = y_plus_e7;
y_plus{8} = y_plus_e8;
y_plus{9} = y_plus_e9;
y_plus{10} = y_plus_e10;
y_plus{11} = y_plus_e11;
y_plus{12} = y_plus_e12;

y_minus{1} = y_minus_e1;
y_minus{2} = y_minus_e2;
y_minus{3} = y_minus_e3;
y_minus{4} = y_minus_e4;
y_minus{5} = y_minus_e5;
y_minus{6} = y_minus_e6;
y_minus{7} = y_minus_e7;
y_minus{8} = y_minus_e8;
y_minus{9} = y_minus_e9;
y_minus{10} = y_minus_e10;
y_minus{11} = y_minus_e11;
y_minus{12} = y_minus_e12;
%%

% The empirical local observability gramian is initialized here:
obs_gram = zeros(states);

for row = 1:states
    for column = 1:states
        %integrand = (y_plus{row} - y_minus{row}).'*(y_plus{column} - y_minus{column});
        a = y_plus{row} - y_minus{row};
        b = y_plus{column} - y_minus{column};
        integrand = a(1, :).*b(1, :) + a(2, :).*b(2, :) + a(3, :).*b(3, :);
        integral = trapz(integrand);
        obs_gram(row, column) = (1/(4*epsilon^2))*integral;
    end
end

obs_gram_eig = eig(obs_gram);

%%  Check the observability

% Local singular values
lsv = sqrt(obs_gram_eig);
% Local unobservability index:
lui = 1/min(lsv)
% Local estimation condition number:
lecn = sqrt(max(obs_gram_eig)/min(obs_gram_eig))


%% 
% Todo:



% Scale coordinates and outputs

% Simulate this: x+e1, x+e2 ... x+e12, then, x-e1, x-e2 ... x-e12
% and store the output trajectories.

% Initialize a 12x12 zero-matrix.
% Calculate all the elements of the empirical local observability gramian
% using formula (4) from the paper inside a nested for loop.
% Place the elements in the initialized matrix after they have been
% calculated.

