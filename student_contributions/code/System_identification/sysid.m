clear
%% Loading system parameters
parameters_maggy_V2;

% Compute radius correction factor
correctionFactorFast = computeSolenoidRadiusCorrectionFactor(params,'fast');

paramsFast = params;
paramsFast.solenoids.r = correctionFactorFast*paramsFast.solenoids.r;

% Computing equilibria
[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(paramsFast,'fast');


%% Setup system 
% Define the model to be used in linearization
modelName = 'fast';

% Setting up system equations with speficied model and parameters for
% simpler representation
f = @(x,u) maglevSystemDynamics(x,u,paramsFast,modelName);
h = @(x,u) x; %maglevSystemMeasurements(x,u,paramsFast,modelName); --> Changing measurements

%% Control design (Initial controller)
% Linearization
xLp = [0,0,zEq,zeros(1,9)]'; % Linearizing around the equilibria
uLp = zeros(length(params.solenoids.r),1);

delta = 1e-6; % Step-size used in numerical linearization
[A,B,C,D] = finiteDifferenceLinearization(f,h,xLp,uLp,delta);

% Defining reduced order system
I = [1:5,7:11];
Ared = A(I,I);
Bred = B(I,:);
Cred = C(:,I);
Dred = D(:,:);

% Cost matrices
Q = diag([1e1, 1e1, 1e1, 1e0, 1e1, 1e0, 1e0, 1e0, 1e0, 1e0]);
R = 1e0*eye(length(params.solenoids.r));

% Computing LQR estimate
Kred = lqr(Ared,Bred,Q,R); % Rounding can sometimes be dangerous!

% increasing order of our controller for controlling the real system
K = [Kred(:,1:5), zeros(4,1), Kred(:,6:end), zeros(4,1)];

%% Simulation setup
x0 = xLp;                       % initial condition
tSpan = linspace(0, 1, 1000);   % interval of integration (array of time points)

%% Reference design
ax = 0.002;     % amplitude in x-direction
ay = 0.002;     % amplitude in y-direction
az = 0;         % amplitude in z-direction

wx = 10;        % angular frequency of the square waves in x-direction
wy = 10;        % angular frequency of the square waves in y-direction
wz = 10;        % angular frequency of the square waves in z-direction

t_start = 0.1;  % initialize the start time for the reference signal

n = 3;          % number of terms in the series

% Define the random trajectory for x-coordinate
% The function uses square waves shifted randomly
xr = @(t) ax*(square(wx*t) + square(wx*t + 2*pi*rand(1)) + square(wx*t + 2*pi*rand(1)))/n; 

% Define the random trajectory for y-coordinate
% The function uses square waves shifted randomly
yr = @(t) ay*(square(wy*t + 2*pi*rand(1)) + square(wy*t + 2*pi*rand(1)) + square(wy*t + 2*pi*rand(1)))/n;

% Define the random trajectory for z-coordinate
% The function uses square waves shifted randomly
% zEq is the equilibrium position in the z-direction
zr = @(t) az*square(wz*t) + zEq;


% Construct the reference vector as a function of time
% The reference vector has 12 components, first three are positions, and the rest are zeros
X =  @(t) ((t_start < t(:)).*[xr(t(:)), yr(t(:)), zr(t(:)), zeros(length(t), 9)])';

% Evaluate the trajectory over a time span tSpan
xd_vec = X(tSpan);

% Interpolate the trajectory vector to get a function of time
% This function will provide the interpolated trajectory at any given time t
xd = @(t) interp1(tSpan',xd_vec',t)';

% Define a function to return the reference vector at time t
r = @(t) xd(t);

%% Defining input with controller
u = @(t,x) K*(r(t)-(x-xLp))-uLp;

%% Simulation
disp('Simulation...')
tic;

options = odeset('Events', @eventsFcn);
[t,x] = ode15s(@(t,x) f(x,u(t,x)), tSpan, x0, options);

fprintf('LQR simulation time: %.2fs\n', toc)

% Plot the state trajectories against their respective references
f7 = figure(7);
clf; grid on; hold on; box on;
plot(t,x(:,1:3), t ,xd_vec(1:3,:), 'linewidth',2)

xlabel('t')
ylabel('x/y/z')
legend({'x','y','z', 'xd','yd','zd'},'location','best')


%% Sampling data
y = x - xLp';
u = u(tSpan,x')';
Ts = max(diff(tSpan));
data = detrend(iddata(y, u, Ts));

%% Plotting the data
f8 = figure();
clf
plot(data);

%% Estimating the state-space model using different subspace method

% The physical model for comparison
sys_real = idss(c2d(ss(A,B,C,D), Ts));


% Model derived using the DSR toolbox by David Di Ruscio.
L = 2;         % number of block rows in extended observability matrix. 
g = 0;         % closed loop systems
J = 10;        % past horizon used to define instruments
n = 12;        % system order 
im = 0;        % handling instrumental variables

[a,b,d,e,c,f,x0,E_J1,DXJ,sn] = dsr_e(y,u,L,g,J,n,im);
sys_dsr  = idss(c2d(ss(a, b, d, e), Ts)); 


% Model derived using the function "n4sid.m" by The MathWorks, Inc
sys_n4sid = n4sid(data, 12, 'Ts', Ts);

% Model derived using the function "moesp.m" by Yi Cao at Cranfield University on 27th April 2008
[~,ssfun] = moesp(y, u, 20);
[A_moesp,B_moesp,C_moesp,D_moesp] = ssfun(12);
sys_moesp = idss(c2d(ss(A_moesp,B_moesp,C_moesp,D_moesp), Ts));

%% Check controllability and observability

% disp('Rank of the controllability matrix: ');
% disp('sys_real:'); disp(rank(ctrb(sys_real)));
% disp('sys_dsr:'); disp(rank(ctrb(sys_dsr)));
% disp('sys_n4sid:'); disp(rank(ctrb(sys_n4sid)));
% disp('sys_moesp:'); disp(rank(ctrb(sys_moesp)));
 
% disp('Rank of observability matrix: ');
% disp('sys_real:'); disp(rank(obsv(sys_real)));
% disp('sys_dsr:'); disp(rank(obsv(sys_dsr)));
% disp('sys_n4sid:'); disp(rank(obsv(sys_n4sid)));
% disp('sys_moesp:'); disp(rank((sys_moesp)));

%% Calculate the eigenvalues of the closed-loop system

% eig_real = eig(sys_real.A - sys_real.B * Ts * K);
% eig_dsr = eig(sys_dsr.A - sys_dsr.B * Ts * K);
% eig_n4sid = eig(sys_n4sid.A - sys_n4sid.B * K);
% eig_moesp = eig(sys_moesp.A - sys_moesp.B * Ts * K);

% disp('Eigenvalues of the closed-loop systems:');
% disp('sys_real:'); disp(eig_real);
% disp('sys_dsr:'); disp(eig_dsr);
% disp('sys_n4sid:'); disp(eig_n4sid);
% disp('sys_moesp:'); disp(eig_moesp);

%% Manual one-step prediction (can only do this because y := h(x) = x)
N = length(tSpan);

% Initialize state estimates
y1 = zeros(N, 12);
y2 = zeros(N, 12);
y3 = zeros(N, 12);
y4 = zeros(N, 12);

% Loop through each time step from 1 to N-1
% K is the controller gain, Ts is the sampling time, r(tSpan(i)) is the reference signal at time tSpan(i)
for i = 1:N-1
    
    % Update the state y1 using the physical system matrices and control input
    y1(i+1,:) = (sys_real.A * y(i,:)' + sys_real.B * (-Ts*K*(y(i,:)' - r(tSpan(i)))))';
    
    % Update the state y2 using the DSR system matrices and control input
    y2(i+1,:) = (sys_dsr.A * y(i,:)' + sys_dsr.B * (-Ts*K*(y(i,:)' - r(tSpan(i)))))';
    
    % Update the state y3 using the N4SID system matrices and control input
    y3(i+1,:) = (sys_n4sid.A * y(i,:)' + sys_n4sid.B * (-Ts*K*(y(i,:)' - r(tSpan(i)))))';
    
    % Update the state y4 using the MOESP system matrices and control input
    y4(i+1,:) = (sys_moesp.A * y(i,:)' + sys_moesp.B * (-Ts*K*(y(i,:)' - r(tSpan(i)))))';
end

% Transpose and offset each state trajectory by the initial condition xLp
y1 = y1' + xLp;
y2 = y2' + xLp;
y3 = y3' + xLp;
y4 = y4' + xLp;

%% Plot the one-step prediction of the physical system 

f9 = figure();
clf; 
subplot(3,1,1);
grid on; hold on; box on;
plot(t, x(:,1:2), 'b-', 'linewidth', 2, 'displayname', 'Data')
plot(t(2:end), x(1:end-1,1:2), 'g-', 'linewidth', 2, 'displayname', 'Shifted one step')
plot(t, y1(1:2,:), 'r--', 'linewidth', 2, 'displayname', 'Prediction')
xlabel('t','fontsize',14)
ylabel('x/y','fontsize',14)
legend('location','best')

subplot(3,1,2);
grid on; hold on; box on;
plot(t, x(:,3), 'b-', 'linewidth', 2)
plot(t(2:end), x(1:end-1,3), 'g-', 'linewidth', 2)
plot(t, y1(3,:), 'r--', 'linewidth', 2)
xlabel('t','fontsize',14)
ylabel('z','fontsize',14)

subplot(3,1,3);
grid on; hold on; box on;
plot(t, x(:,4:6), 'b-', 'linewidth', 2)
plot(t(2:end), x(1:end-1,4:6), 'g-', 'linewidth', 2)
plot(t, y1(4:6,:), 'r--', 'linewidth', 2)
xlabel('t','fontsize',14)
ylabel('a/b/c','fontsize',14)


%% Plot the one-step prediction of the DSR system 

f10 = figure();
clf; 
subplot(3,1,1);
grid on; hold on; box on;
plot(t, x(:,1:2), 'b-', 'linewidth', 2, 'displayname', 'Data')
plot(t(2:end), x(1:end-1,1:2), 'g-', 'linewidth', 2, 'displayname', 'Shifted one step')
plot(t, y2(1:2,:), 'r--', 'linewidth', 2, 'displayname', 'Prediction')
xlabel('t','fontsize',14)
ylabel('x/y','fontsize',14)
legend('location','best')

subplot(3,1,2);
grid on; hold on; box on;
plot(t, x(:,3), 'b-', 'linewidth', 2)
plot(t(2:end), x(1:end-1,3), 'g-', 'linewidth', 2)
plot(t, y2(3,:), 'r--', 'linewidth', 2)
xlabel('t','fontsize',14)
ylabel('z','fontsize',14)

subplot(3,1,3);
grid on; hold on; box on;
plot(t, x(:,4:6), 'b-', 'linewidth', 2)
plot(t(2:end), x(1:end-1,4:6), 'g-', 'linewidth', 2)
plot(t, y2(4:6,:), 'r--', 'linewidth', 2)
xlabel('t','fontsize',14)
ylabel('a/b/c','fontsize',14)


%% Plot the one-step prediction of the N4SID system 

f11 = figure();
clf; 
subplot(3,1,1);
grid on; hold on; box on;
plot(t, x(:,1:2), 'b-', 'linewidth', 2, 'displayname', 'Data')
plot(t(2:end), x(1:end-1,1:2), 'g-', 'linewidth', 2, 'displayname', 'Shifted one step')
plot(t, y3(1:2,:), 'r--', 'linewidth', 2, 'displayname', 'Prediction')
xlabel('t','fontsize',14)
ylabel('x/y','fontsize',14)
legend('location','best')

subplot(3,1,2);
grid on; hold on; box on;
plot(t, x(:,3), 'b-', 'linewidth', 2)
plot(t(2:end), x(1:end-1,3), 'g-', 'linewidth', 2)
plot(t, y3(3,:), 'r--', 'linewidth', 2)
xlabel('t','fontsize',14)
ylabel('z','fontsize',14)

subplot(3,1,3);
grid on; hold on; box on;
plot(t, x(:,4:6), 'b-', 'linewidth', 2)
plot(t(2:end), x(1:end-1,4:6), 'g-', 'linewidth', 2)
plot(t, y3(4:6,:), 'r--', 'linewidth', 2)
xlabel('t','fontsize',14)
ylabel('a/b/c','fontsize',14)


%% Plot the one-step prediction of the MOESP system 

f12 = figure();
clf; 
subplot(3,1,1);
grid on; hold on; box on;
plot(t, x(:,1:2), 'b-', 'linewidth', 2, 'displayname', 'Data')
plot(t(2:end), x(1:end-1,1:2), 'g-', 'linewidth', 2, 'displayname', 'Shifted one step')
plot(t, y4(1:2,:), 'r--', 'linewidth', 2, 'displayname', 'Prediction')
xlabel('t','fontsize',14)
ylabel('x/y','fontsize',14)
legend('location','best')

subplot(3,1,2);
grid on; hold on; box on;
plot(t, x(:,3), 'b-', 'linewidth', 2)
plot(t(2:end), x(1:end-1,3), 'g-', 'linewidth', 2)
plot(t, y4(3,:), 'r--', 'linewidth', 2)
xlabel('t','fontsize',14)
ylabel('z','fontsize',14)

subplot(3,1,3);
grid on; hold on; box on;
plot(t, x(:,4:6), 'b-', 'linewidth', 2)
plot(t(2:end), x(1:end-1,4:6), 'g-', 'linewidth', 2)
plot(t, y4(4:6,:), 'r--', 'linewidth', 2)
xlabel('t','fontsize',14)
ylabel('a/b/c','fontsize',14)


%%  Calculate the Variance Accounted For (VAF) percentage by comparing the measured output y with the closed-loop respone of the various subspace methods
vaf1 = vaf(y, y1);  % VAF between measured output y and closed-loop response of the physical system 
vaf2 = vaf(y, y2);  % VAF between measured output y and closed-loop response of the DSR system 
vaf3 = vaf(y, y3);  % VAF between measured output y and closed-loop response of the N4SID system 
vaf4 = vaf(y, y4);  % VAF between measured output y and closed-loop response of the MOESP system 


%% Additional functions
function [value,isterminal,direction] = eventsFcn(t,x) % Stop simulation if control is unstable
    value = min(0.02 - abs(x(1:2)));

    isterminal = 1;
    direction = 0;
end