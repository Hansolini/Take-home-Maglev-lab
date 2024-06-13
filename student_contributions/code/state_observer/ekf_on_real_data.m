clc; clear all;

%% Load data
load('data\PD_data_with_input.mat');

%% Format data
% Index for only sampling a single snippet of data
I = 10000:11250;

% Metadata
n = length(I);          % Number samples
freq = 2700;            % Sampling frequency

scaling_output = 1/1000;      % Scaling factor for matching real output to model
scaling_input  = 1/1000;    % Scaling factor for matching real input to model

% Data
t = linspace(0,n/freq,n);
Y = scaling_output*[rawMeasurements1(I,:), rawMeasurements2(I,:), rawMeasurements3(I,:)].*[1,1,1,-1,1,-1,-1,1,-1];
U = scaling_input*[u(I,1), u(I,2), -u(I,1), -u(I,2)];

% Model initialization
% Load parameters
parameters_maggy_V2;

% Model selection and parameter correction
modelName = 'fast';
correctionFactorFast = 0.558361865282244;
paramsFast = params;
paramsFast.solenoids.r = correctionFactorFast*paramsFast.solenoids.r;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Modifying parameters
paramsFast.magnet.m = 0.075;
paramsFast.magnet.r = 0.025;
paramsFast.permanent.l = 2*0.004*ones(size(params.permanent.l));

index_optimization = [
  1;    % Permanent xy
  %2;    % Sensor xy
  %3;    % Magnet m 
  %4;    % Magnet J
  %5;    % Magnet l
  %6;    % Magnet r
  %7;    % Permanent J
  8;    % Permanent l
  9;    % Permanent r
];

p0 = [
        max(paramsFast.permanent.x);
        max(paramsFast.sensors.x);
        paramsFast.magnet.m;
        paramsFast.magnet.J;
        paramsFast.magnet.l;
        paramsFast.magnet.r;
        paramsFast.permanent.J;
        paramsFast.permanent.l(1);
        paramsFast.permanent.r(1);
      ];

disp(objfun(p0(index_optimization),paramsFast,modelName,mean(Y),index_optimization))
%
options = optimoptions('fminunc', 'Display', 'iter');
%p = fminunc(@(p) objfun(p,paramsFast,modelName,mean(Y),index_optimization), p0(index_optimization),options);
p = [
   0.0275;
   0.0069;
   0.0131;
   ];
for i = 1:length(index_optimization)
    switch index_optimization(i)
        case 1
            paramsFast.permanent.x = p(i)*sign(params.permanent.x);
            paramsFast.permanent.y = p(i)*sign(params.permanent.y);
        case 2
            paramsFast.sensors.x = p(i)*sign(params.sensors.x);
            paramsFast.sensors.y = p(i)*sign(params.sensors.y);
        case 3
            paramsFast.magnet.m = p(i);
        case 4
            paramsFast.magnet.J = p(i);
        case 5
            paramsFast.magnet.l = p(i);
        case 6
            paramsFast.magnet.r = p(i);
        case 7
            paramsFast.permanent.J = p(i);
        case 8
            paramsFast.permanent.l = p(i)*ones(size(params.permanent.l)); 
            paramsFast.permanent.z = p(i)/2*ones(size(paramsFast.permanent.z));
        case 9
            paramsFast.permanent.r = p(i)*ones(size(params.permanent.r));
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize system functions
f = @(x,u) maglevSystemDynamics(x,u,paramsFast,modelName);
h = @(x,u) maglevSystemMeasurements(x,u,paramsFast,modelName);

% Model linearization
% Find system equilibria
[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(paramsFast,modelName);

% Define linearization point
xLp = [0,0,zEq(1),zeros(1,9)]'; % Linearizing around the equilibria
uLp = zeros(length(params.solenoids.r),1);
yLp = h(xLp,uLp);

% Plotting data
% Output data
figure(1);
clf; 

subplot(3,1,1);
grid on; hold on;
plot(t,Y(:,1:3),'linewidth',2)

yline(yLp(1:3),'linewidth',2)

title('Sensor 1')
xlabel('Time [s]')
ylabel('Measurement [T]')

xlim([min(t), max(t)])
ylim([-0.02, 0.02])

subplot(3,1,2);
grid on; hold on;
title('Sensor 2')
xlabel('Time [s]')
ylabel('Measurement [T]')

plot(t,Y(:,4:6),'linewidth',2)
yline(yLp(4:6),'linewidth',2)

xlim([min(t), max(t)])

subplot(3,1,3);
grid on; hold on;
title('Sensor 3')
xlabel('Time [s]')
ylabel('Measurement [T]')

plot(t,Y(:,7:9),'linewidth',2)
yline(yLp(7:9),'linewidth',2)

xlim([min(t), max(t)])

% Input data
figure(2);
clf; grid on; hold on;
plot(t,U,'linewidth',2)

%% EKF observer
% Define initial state estimate and covariance
P = 1e-1*eye(length(xLp)); % Initial covariance estimate

% Define process and measurement noise covariance matrices
Q_EKF = 1e-2*eye(length(xLp)); % Process noise covariance
R_EKF = 1e1*eye(length(yLp)); % Measurement noise covariance

% Simulate EKF
tic
x_est_ekf = ekf(t,U,Y,f,h,xLp,Q_EKF,R_EKF,P);
fprintf('EKF simulation time: %.2fs\n', toc)

% Get output
Y_est = zeros(size(Y));
for i = 1:length(t)
    Y_est(i,:) = h(x_est_ekf(i,:)',U(i,:)')';
end
%% Plotting EKF results
figure(3);
clf; grid on; hold on;
plot(t,x_est_ekf(:,1:3),'linewidth',2)

figure(4);
clf; grid on; hold on;
plot(t,Y(:,1:9),'b','LineWidth',2)
plot(t,Y_est(:,1:9),'r--','LineWidth',2)
%% Functions: Optimization of system parameters
function val = objfun(p,params,modelName,yLp_real,index_optimization)
    % Update parameters
    for i = 1:length(index_optimization)
        switch index_optimization(i)
            case 1
                params.permanent.x = p(i)*sign(params.permanent.x);
                params.permanent.y = p(i)*sign(params.permanent.y);
            case 2
                params.sensors.x = p(i)*sign(params.sensors.x);
                params.sensors.y = p(i)*sign(params.sensors.y);
            case 3
                params.magnet.m = p(i);
            case 4
                params.magnet.J = p(i);
            case 5
                params.magnet.l = p(i);
            case 6
                params.magnet.r = p(i);
            case 7
                params.permanent.J = p(i);
            case 8
                params.permanent.l = p(i)*ones(size(params.permanent.l)); 
                params.permanent.z = 0.5*p(i)*ones(size(params.permanent.l));
            case 9
                params.permanent.r = p(i)*ones(size(params.permanent.r));
        end
    end
    % Load measurement function
    h = @(x,u) maglevSystemMeasurements(x,u,params,modelName);
    
    % Find system equilibria
    [zEq, ~, ~, ~] = computeSystemEquilibria(params,modelName);
    
    if isempty(zEq)
        val = inf;
    else
        % Compute output at equilibrium
        xLp = [0,0,zEq(1),zeros(1,9)]'; % Linearizing around the equilibria
        uLp = zeros(length(params.solenoids.r),1);
        yLp = h(xLp,uLp);
        
        % Update objectuve function
        val = norm(yLp(:) - yLp_real(:));
    end
end

%% Functions: EKF
function x_est = ekf(t,U,Y,f,h,x0,Q,R,P)
    % Define discrete system functions
    dt = min(diff(t));
    f_disc = @(x,u) x + dt*f(x,u);
    h_disc = @(x,u) h(x,u);
    
    % Define Jacobian functions for EKF
    delta = 1e-6;
    F = @(x, u) finiteDifferenceJacobian(f_disc, x, u, delta);
    H = @(x, u) finiteDifferenceJacobian(h_disc, x, u, delta);
    
    % Index to reduce order of system (to remove unobservable states)
    I = [1,2,3,4,5,7,8,9,10,11];
    Qred = Q(I,I);
    Rred = R;
    Pred = P(I,I);

    % Number of measurements
    num_measurements = size(Y, 1);
    
    % Initialize array to store the state estimates
    x_est = zeros(num_measurements,length(x0));
    x_est(1,:) = x0';
    
    for k = 1:num_measurements-1
        % Update message
        message = sprintf('EKF completeness: %.0f %%\n', 100*k/num_measurements);        
        
        if k > 1
            fprintf(repmat('\b', 1, length_prev_message));
        end
        fprintf('%s', message);

        length_prev_message = length(message);

        % Prediction step
        F_k = F(x_est(k,:)', U(k,:)');            % Jacobian of f at x_est
        F_k_red = F_k(I,I);

        x_pred = f_disc(x_est(k,:)', U(k,:)');    % Predicted state estimate
        x_pred_red = x_pred(I);                   % Reduced predicted state
        P_pred = F_k_red*Pred*F_k_red' + Qred;    % Predicted covariance estimate

        % Update step
        H_k = H(x_pred, U(k,:)');                 % Jacobian of h at x_pred
        H_k_red = H_k(:,I);

        % Use pseudo-inverse instead of regular inverse
        S = H_k_red*P_pred*H_k_red' + Rred;
        K_red = P_pred*H_k_red' * pinv(S);    % Kalman gain for reduced system

        % Update only the observable states
        x_est_red = x_pred_red + K_red*(Y(k,:)' - h_disc(x_pred, U(k,:)')); % Updated reduced state estimate
        x_est(k+1,I) = x_est_red';                                     % Assign updated reduced state to full state estimate
        x_est(k+1,setdiff(1:length(x0), I)) = 0; % Set unobservable states to 0

        % Update the covariance matrix
        Pred = (eye(size(Pred)) - K_red*H_k_red)*P_pred; % Updated reduced covariance estimate
    end
end

function J = finiteDifferenceJacobian(func, x, u, delta)
    n = length(x);
    m = length(func(x, u));
    J = zeros(m, n);
    for i = 1:n
        dx = zeros(n, 1);
        dx(i) = delta;
        J(:, i) = (func(x + dx, u) - func(x - dx, u))/(2*delta);
    end
end