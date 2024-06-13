clear all; clc; %close all;
%{
%% Luenberger observer
%}

%% System initialization
% Load parameters
parameters_maggy_V2;

% Model selection and parameter correction
modelName = 'fast';
correctionFactorFast = 0.558361865282244;
paramsFast = params;
paramsFast.solenoids.r = correctionFactorFast*paramsFast.solenoids.r;

% Initialize system functions
f = @(x,u) maglevSystemDynamics(x,u,paramsFast,modelName);
h = @(x,u) maglevSystemMeasurements(x,u,paramsFast,modelName);

%% Model linearization
% Find system equilibria
[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(paramsFast,'fast');

% Define linearization point
xLp = [0,0,zEq(1),zeros(1,9)]'; % Linearizing around the equilibria
uLp = zeros(length(params.solenoids.r),1);
yLp = h(xLp,uLp);

% Linearization
delta = 1e-6; % Step-size used in numerical linearization
[A,B,C,D] = finiteDifferenceLinearization(f,h,xLp,uLp,delta);

%% Construct LQR controller
% Defining reduced order system (to avoid unctr/unobs states)
I = [1:5,7:11];
Ared = A(I,I);
Bred = B(I,:);
Cred = C(:,I);
Dred = D(:,:);

% Cost matrices
Q = diag([1e6,1e6,1e2, 1e1,1e1, 1e2,1e2,1e2, 1e2,1e2]);
R = 1e-0*eye(length(params.solenoids.r));

% Computing LQR estimate
Kred = round(lqr(Ared,Bred,Q,R),3); % Rounding can sometimes be dangerous!

% Increasing control order
K = [Kred(:,1:5), zeros(4,1), Kred(:,6:end), zeros(4,1)];

% Define controller function
u = @(t,x) -K*(x-xLp)-uLp;

%% Simulation
% Setup
x0 = xLp + [-0.002 0.004 0.03 0 pi/5 0 zeros(1, 6)].';
tspan = linspace(0,1,2000);
threshold = 0.02;

% Define options with custom output function
options = odeset('OutputFcn', @(t, x, flag) checkStateNorm(t, x, flag, threshold),'RelTol',1e-8,'AbsTol',1e-6);

% Simulation
tic
[t,x] = ode15s(@(t,x) f(x,u(t,x)), tspan, x0, options);
fprintf('LQR simulation time: %.2fs\n', toc)

% Compute output
Y = zeros(length(t), length(h(xLp,uLp)));
U = zeros(length(t), length(uLp));
for i = 1:length(t)
    U(i,:) = u(t(i), x(i,:)')';
    Y(i,:) = h(x(i,:)', U(i,:)')' + 0.001*(rand(1,9)-0.5);
end

%% Luenberger observer
% Desired observer poles (should be faster than system poles)
desired_poles = linspace(-300,-50,10);

% Compute the Luenberger gain matrix L
Lred = place(Ared', Cred', desired_poles)';

% Increase the order of the matrix
L = [Lred(1:5,:); zeros(1,9); Lred(6:end,:); zeros(1,9)];

% Simulate observer
tic
[t_est,x_est] = ode15s(@(t_est,x_est) f_obs(x_est,f,h,interp1(t,Y,t_est)',u(t_est,interp1(t,x,t_est)'),L), tspan, xLp, options);
fprintf('Observer simulation time: %.2fs\n', toc)

%% EKF observer
% Define initial state estimate and covariance
P = 1e-1*eye(length(xLp)); % Initial covariance estimate

% Define process and measurement noise covariance matrices
Q_EKF = 1e-4*eye(length(xLp)); % Process noise covariance
R_EKF = 1e-3*eye(length(yLp)); % Measurement noise covariance

% Simulate EKF
tic
x_est_ekf = ekf(t,U,Y,f,h,xLp,Q_EKF,R_EKF,P);
fprintf('EKF simulation time: %.2fs\n', toc)

%% Figure
figure(1);
clf; 
subplot(3,1,1);
grid on; hold on; box on;
plot(t,x(:,1:3), 'linewidth', 2, 'displayname', 'Ground truth')
plot(t_est,x_est(:,1:3), '--', 'linewidth', 2, 'displayname', 'Luenberger')
plot(t,x_est_ekf(:,1:3), ':', 'linewidth', 2, 'displayname', 'EKF')

xlim([min(t), max(t)])
xlabel('t')
ylabel('x/y/z')
legend('location','best')

subplot(3,1,2);
grid on; hold on; box on;
plot(t,x(:,4:5), 'linewidth', 2, 'displayname', 'Ground truth')
plot(t_est,x_est(:,4:5), '--', 'linewidth', 2, 'displayname', 'Luenberger')
plot(t,x_est_ekf(:,4:5), ':', 'linewidth', 2, 'displayname', 'EKF')

xlim([min(t), max(t)])
ylim(1.1*[min(min(x(:,4:5))), max(max(x(:,4:5)))])
xlabel('t')
ylabel('a/b')
legend('location','best')

subplot(3,1,3);
grid on; hold on; box on;

if length(t_est) == length(t)
    plot(t,vecnorm((x - x_est)'), '--', 'linewidth', 2, 'displayname', 'Luenberger')
    plot(t,vecnorm((x - x_est_ekf)'), ':', 'linewidth', 2, 'displayname', 'EKF')
end

xlim([min(t), max(t)])
xlabel('t')
ylabel('||error||')
legend('location','best')

%% 
figure(3);
clf; grid on; hold on;
%plot(1000*Y(:,[1,4,7]+1),'linewidth', 2)
plot(t,x(:,7:9),'Linewidth',2)
plot(t,x_est_ekf(:,7:9),'--','Linewidth',2)

%% Functions: Simulation
function status = checkStateNorm(t, x, flag, threshold)
    % Custom output function to stop the simulation if the norm of the first
    % two states exceeds the threshold

    if strcmp(flag, 'init')
        % Initialization
        status = 0;
        return;
    elseif strcmp(flag, 'done')
        % Cleanup
        status = 0;
        return;
    else
        % During integration
        if norm(x(1:2)) > threshold
            status = 1; % Stop the integration
            return;
        end
    end

    status = 0; % Continue the integration
end

%% Functions: Leuenberger
function dx_est = f_obs(x_est,f,h,y_true,u_true,L)
    y_est  = h(x_est,u_true);
    dx_est = f(x_est,u_true) + L*(y_true - y_est);

    dx_est([6, 12]) = 0; % Forcing z-rotation to be 0
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

