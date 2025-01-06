%% Load the parameters "params"
parameters_maggy_V2;
%%
params.magnet.m = 0.02;

% Compute radius correction factor
cFast = computeSolenoidRadiusCorrectionFactor(params,'fast');

paramsFast = params;
paramsFast.solenoids.r = cFast*paramsFast.solenoids.r;

% Compute system equilibria
[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(paramsFast,'fast');

clc;
disp(zEq)
disp(zEqInv)

%% Compute z curve
% Defining range
zMin = max([params.solenoids.z+params.solenoids.l/2, params.permanent.z+params.permanent.l/2]) + params.magnet.l/2; % Physical Limitations
zMax = 0.1; % Arbitrary
z = linspace(zMin,zMax,100);

paramsCorrected = params;
paramsCorrected.solenoids.r = 0.5584*paramsCorrected.solenoids.r;
paramsCorrected.magnet.J = -paramsCorrected.magnet.J;
paramsCorrected.magnet.m = 0.02;

% Computing
fz = computeVerticalForce(z,paramsCorrected,'fast');
%fza = computeVerticalForce(z,paramsCorrected,'accurate');
%fzf = computeVerticalForce(z,params,'fillament');

% Plotting
figure(1);
clf; grid on; hold on;
plot(z,fz,'b','linewidth',2)
%plot(z,fza,'r--','linewidth',2)
%plot(z,fzf,'k:','linewidth',2)
yline(0,'linewidth',2)



%% Setup system
modelName = 'fast';

% Set up the system equation
f = @(x,u) maglevSystemDynamics(x,u,paramsFast,modelName);

% Set up the measurement equation
h = @(x,u) maglevSystemMeasurements(x,u,paramsFast,modelName);

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
xLp_inv = [0,0,max(zEqInv),pi,zeros(1,8)]';
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
x0 = xLp;
t_span = linspace(0,1,600);

t_start  = 0.2;
t_turn   = t_start + 0.04;
t_pause  = t_start + 0.047;
t_switch = t_start + 0.047;

r_before = @(t) (t > t_start)*[0, -0.01, 0.3,  pi*(t > t_turn), 0, 0, 0, zeros(1,5)]';
r_after  = @(t) zeros(12,1);

u_before = @(t,x) K*(r_before(t)-(x-xLp))-uLp;
u_after  = @(t,x) K_inv*(r_after(t)-(x-xLp_inv))-uLp;

u = @(t,x) (t < t_pause)*u_before(t,x) + (t > t_switch)*u_after(t,x);
%u = @(t,x) u_after(t,x);

% Simulation
disp('Simulation...')
tic;

options = odeset('Events', @eventsFcn);
[t,x] = ode15s(@(t,x) f(x,u(t,x)), t_span, x0, options);
toc

%% Figure 
figure(1);
clf; grid on; hold on; box on; daspect([1,1,1]);
view([45,15]);

xlim([-0.07,0.07])
ylim([-0.07,0.07])
zlim([-0,0.12])
plotBaseArtistic(params);
H = plotMagnetArtistic(0,0,0.05,0,0.5,0.5,params);

xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')


filename = 'magnet_flipping.gif';

for i = 1:length(t)
    disp(i)
    title(sprintf('Magnet flipping\n t = %.2f s', t(i)))
    updatePositionOfObject(H,x(i,1),x(i,2),x(i,3),x(i,4),x(i,5),x(i,6));
    drawnow;
    if 1
        % Capture the plot as an image
        frame = getframe(gcf);
        img = frame2im(frame);
        [A,map] = rgb2ind(img,256); 
        
        % Write to the GIF File 
        if i == 1
            imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',1/60);
        else
            imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',1/60);
        end
    end
end

%%
figure(2);
clf; 

subplot(2,1,1);
grid on; hold on; box on;
plot(t, x(:,1:3), 'linewidth', 2)

xline(t_turn, 'k--', 'linewidth', 2)
xline(t_pause, 'k--', 'linewidth', 2)
xline(t_switch, 'k--', 'linewidth', 2)

subplot(2,1,2);
grid on; hold on; box on;
plot(t, x(:,4:6), 'linewidth', 2)

xline(t_turn, 'k--', 'linewidth', 2)
xline(t_pause, 'k--', 'linewidth', 2)
xline(t_switch, 'k--', 'linewidth', 2)

ylim([-pi, pi])

%% Additional functions
function [value,isterminal,direction] = eventsFcn(t,x)
    value = min(0.02 - abs(x(1:2)));
    
    %disp(value)

    isterminal = 1;
    direction = 0;
end