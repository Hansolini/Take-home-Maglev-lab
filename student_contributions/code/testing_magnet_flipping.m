%% Load the parameters "params"
parameters_maggy_V2;

%% Compute radius correction factor
cFast = computeSolenoidRadiusCorrectionFactor(params,'fast');
cAccurate = computeSolenoidRadiusCorrectionFactor(params,'fast');

paramsFast = params;
paramsFast.solenoids.r = cFast*paramsFast.solenoids.r;

paramsAccurate = params;
paramsAccurate.solenoids.r = cAccurate*paramsAccurate.solenoids.r;

%% Compute system equilibria
[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(params,'fast');

%% Setup system
modelName = 'fast';

% Set up the system equation
f = @(x,u) maglevSystemDynamics(x,u,params,modelName);

% Set up the measurement equation
h = @(x,u) maglevSystemMeasurements(x,u,params,modelName);

%% Control design (Initial controller)
% Linearization
xLp = [0,0,zEq,zeros(1,9)]';
uLp = zeros(length(params.solenoids.r),1);

delta = 1e-6;
[A,B,C,D] = finiteDifferenceLinearization(f,h,xLp,uLp,delta);

% LQR design
% - Two of the states are uncontrollable, so we need to reduce the system
% matrices to compute the LQR gain
Ared = A([1:5,7:11],[1:5,7:11]);
Bred = B([1:5,7:11],:);
Cred = C(:,[1:5,7:11]);
Dred = D(:,:);

% Cost matrices
Q = diag([1e6,1e6,1e6, 1e1,1e1, 1e2,1e2,1e3, 1e2,1e2]);
R = 1e-0*eye(length(params.solenoids.r));

% Computing LQR estimate
Kred = round(lqr(Ared,Bred,Q,R),3); % Rounding can sometimes be dangerous!
K = [Kred(:,1:5), zeros(4,1), Kred(:,6:end), zeros(4,1)];

%% Control design (Inverted controller)
% Linearization
xLp_inv = [0,0,zEq,pi,zeros(1,8)]';
uLp_inv = zeros(length(params.solenoids.r),1);

delta = 1e-6;
[A_inv,B_inv,C_inv,D_inv] = finiteDifferenceLinearization(f,h,xLp_inv,uLp_inv,delta);

% LQR design
% - Two of the states are uncontrollable, so we need to reduce the system
% matrices to compute the LQR gain
Ared_inv = A_inv([1:5,7:11],[1:5,7:11]);
Bred_inv = B_inv([1:5,7:11],:);
Cred_inv = C_inv(:,[1:5,7:11]);
Dred_inv = D_inv(:,:);

% Computing LQR estimate
Kred_inv = round(lqr(Ared_inv,Bred_inv,Q,R),3); % Rounding can sometimes be dangerous!
K_inv = [Kred_inv(:,1:5), zeros(4,1), Kred_inv(:,6:end), zeros(4,1)];

%% Simulation setup
x0 = xLp+[0 0 0 0 0 0 zeros(1, 6)].';
t_span = linspace(0,1,100);

r = @(t) [0, 0, 0.02*sin(30*t),  pi/2*(t > 0.12), 0, 0, 0, zeros(1,5)]';
u = @(t,x) K*(r(t)-(x-xLp))-uLp;

% Simulation
disp('Simulation...')
tic;

options = odeset('Events', @eventsFcn);
[t,x] = ode15s(@(t,x) f(x,u(t,x)), t_span, x0, options);
toc
% Figure 
figure(1);
clf; grid on; hold on; box on; daspect([1,1,1]);
view([45,15]);

xlim([-0.1,0.1])
ylim([-0.1,0.1])
zlim([-0.1,0.1])
plotBaseArtistic(params);
H = plotMagnetArtistic(0,0,0.05,0,0.5,0.5,params);

for i = 1:length(t)
   updatePositionOfObject(H,x(i,1),x(i,2),x(i,3),x(i,4),x(i,5),x(i,6));
   drawnow;
end
%
figure(2);
clf; 

subplot(2,1,1);
grid on; hold on; box on;
plot(t, x(:,1:3), 'linewidth', 2)

subplot(2,1,2);
grid on; hold on; box on;
plot(t, x(:,4:6), 'linewidth', 2)

function [value,isterminal,direction] = eventsFcn(t,x)
    value = min(0.02 - abs(x(1:2)));
    
    %disp(value)

    isterminal = 1;
    direction = 0;
end