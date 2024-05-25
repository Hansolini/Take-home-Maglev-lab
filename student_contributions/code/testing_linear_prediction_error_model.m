rng(7);

%% Load the parameters "params"
parameters_maggy_V2;

% Compute radius correction factor
% cFast = computeSolenoidRadiusCorrectionFactor(params,'fast');
cFast = 0.5584;

paramsFast = params;
paramsFast.solenoids.r = cFast*paramsFast.solenoids.r;

% Compute system equilibria
% [zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(paramsFast,'fast');
zEq = 0.03654;

%% Setup system
modelName = 'fast';

% Set up the system equation
f = @(x,u) maglevSystemDynamics(x,u,paramsFast,modelName);

% Set up the measurement equation
h = @(x,u) x; %maglevSystemMeasurements(x,u,paramsFast,modelName); --> Changing measurements

%% Control design (Initial controller)
% Linearization
xLp = [0,0,zEq,zeros(1,9)]';
uLp = zeros(length(params.solenoids.r),1);

delta = 1e-6;
[A,B,C,D] = finiteDifferenceLinearization(f,h,xLp,uLp,delta);

% Reducing model order
Ared = A([1:5,7:11],[1:5,7:11]);
Bred = B([1:5,7:11],:);
Cred = C(:,[1:5,7:11]);
Dred = D(:,:);

% Construct control cost matrices
Q = diag([1e7,1e7,1e7, 1e1,1e1, 1e6,1e6,1e4, 1e1,1e1]);
R = 1e-0*eye(length(params.solenoids.r));

% Computing LQR gain
Kred = round(lqr(Ared,Bred,Q,R),3); % Rounding can sometimes be dangerous!
K = [Kred(:,1:5), zeros(4,1), Kred(:,6:end), zeros(4,1)];


%% Simulation setup
x0 = xLp;
t_span = linspace(0,1,50);

%% Reference design
n = 3;

t_start = 0.3;

ax = 0.01;
ay = 0.01;
az = 0.01;
wx = 20;
wy = 20;
wz = 10;

xr = @(t) ax*(square(wx*t) + square(wx*t + 2*pi*rand(1)) + square(wx*t + 2*pi*rand(1)))/n;
yr = @(t) ay*(square(wy*t + 2*pi*rand(1)) + square(wy*t + 2*pi*rand(1)) + square(wy*t + 2*pi*rand(1)))/n;
zr = @(t) az*square(wz*t);
X  = @(t) ((t_start < t(:)).*[xr(t(:)), yr(t(:)), zr(t(:)), 0*sin(t(:)), 0*cos(t(:)), zeros(length(t), 7)])';

xd_vec = X(t_span);
xd = @(t) interp1(t_span',xd_vec',t)';

%r = @(t) [0.001*cos(10*t), zeros(length(t),1), 0.01*sin(25*t), zeros(length(t),9)]';
r = @(t) xd(t);

%% Defining input with controller
u = @(t,x) K*(r(t)-(x-xLp))-uLp;

%% Simulation
disp('Simulation...')
tic;

options = odeset('Events', @eventsFcn);
[t,x] = ode15s(@(t,x) f(x,u(t,x)), t_span, x0, options);


% UNCOMMENT FOR LINEAR SIMULATION DATA
%xLp = 0*xLp;
%x0 = xLp;
%u = @(t,x) K*(r(t)-(x-xLp))-uLp;
%[t,x] = ode15s(@(t,x) A*x+B*u(t,x), t_span, x0, options);

toc

%% Sampling data
Y = x - xLp';
U = u(t_span,x')';

Ts = max(diff(t_span));
data = iddata(Y,U,Ts);

%% Constructing linear model
sys = idss(c2d(ss(A,B,C,D),Ts));

%% Figure
R = r(t_span) + xLp;

% Plotting sample data
figure(1);
clf; 

subplot(3,1,1);
grid on; hold on; box on;
plot(t, x(:,1:2), 'g-', 'linewidth', 2)
plot(t, R(1:2,:), 'k--', 'linewidth', 1.5)
xlabel('t','fontsize',14)
ylabel('x/y','fontsize',14)

subplot(3,1,2);
grid on; hold on; box on;
plot(t, x(:,3), 'g-', 'linewidth', 2)
plot(t, R(3,:), 'k--', 'linewidth', 1.5)
xlabel('t','fontsize',14)
ylabel('z','fontsize',14)

subplot(3,1,3);
grid on; hold on; box on;
plot(t, x(:,4:6), 'r-', 'linewidth', 2)
xlabel('t','fontsize',14)
ylabel('a/b/g','fontsize',14)

% Plotting compare
%opt = compareOptions('InitialCondition','z');
%ylin = compare(data,sys,1,opt);
%ylin = ylin.OutputData' + xLp;

%ylin = predict(sys,data,1,opt);
%ylin = ylin.OutputData' + xLp;

%% Manual one-step prediction (can only do this because y := h(x) = x)
N = length(t_span);
ylin = zeros(N, 12);
for i = 1:N-1
    ylin(i+1,:) = (sys.A*Y(i,:)' + sys.B*(-Ts*K*(r(t_span(i)) - Y(i,:)')))'; % Discretization of the controller is approximate
end
ylin = ylin' + xLp;

%%
figure(2);
clf; 

subplot(3,1,1);
grid on; hold on; box on;
plot(t, x(:,1:2), 'b-', 'linewidth', 2, 'displayname', 'Data')
plot(t(2:end), x(1:end-1,1:2), 'g-', 'linewidth', 2, 'displayname', 'Shifted one step')
plot(t, ylin(1:2,:), 'r--', 'linewidth', 2, 'displayname', 'Prediction')
xlabel('t','fontsize',14)
ylabel('x/y','fontsize',14)
legend('location','best')

subplot(3,1,2);
grid on; hold on; box on;
plot(t, x(:,3), 'b-', 'linewidth', 2, 'displayname', 'Data')
plot(t(2:end), x(1:end-1,3), 'g-', 'linewidth', 2, 'displayname', 'Shifted one step')
plot(t, ylin(3,:), 'r--', 'linewidth', 2, 'displayname', 'Prediction')
xlabel('t','fontsize',14)
ylabel('z','fontsize',14)
legend('location','best')

subplot(3,1,3);
grid on; hold on; box on;
plot(t, x(:,4:6), 'b-', 'linewidth', 2, 'displayname', 'Data')
plot(t(2:end), x(1:end-1,4:6), 'g-', 'linewidth', 2, 'displayname', 'Shifted one step')
plot(t, ylin(4:6,:), 'r--', 'linewidth', 2, 'displayname', 'Prediction')
xlabel('t','fontsize',14)
ylabel('a/b/c','fontsize',14)
legend('location','best')

%% Additional functions
function [value,isterminal,direction] = eventsFcn(t,x) % Stop simulation if control is unstable
    value = min(0.02 - abs(x(1:2)));

    isterminal = 1;
    direction = 0;
end