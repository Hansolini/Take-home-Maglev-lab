%% Load the parameters "params"
parameters_maggy_V2;

%% Setup system
modelName = 'accurate';

% Set up the system equation
f = @(x,u) maglevSystemDynamics(x,u,params,modelName);

% Set up the measurement equation
h = @(x,u) maglevSystemMeasurements(x,u,params,modelName);

%% Computation
% Linearization
zEq = 0.0370;

xLp = [0,0,zEq,zeros(1,9)]';
uLp = zeros(length(params.solenoids.r),1);

delta = 1e-5;
[A,B,C,D] = finiteDifferenceLinearization(f,h,xLp,uLp,delta);

% Balancing linearized model
sys = ss(A,B,C,[]);
sys_balanced = balreal(sys);

A_balanced = sys_balanced.A;
B_balanced = sys_balanced.B;
C_balanced = sys_balanced.C;

% Compute obs/ctrb matrices
OB = obsv(A,C);
CO = ctrb(A,B);

OB_balanced = obsv(A_balanced,C_balanced);
CO_balanced = ctrb(A_balanced,B_balanced);

% SVD
S_CO = svd(CO);
S_CO_balanced = svd(CO_balanced);

S_OB = svd(OB);
S_OB_balanced = svd(OB_balanced);

%% Figure
% Text setup
set(0, 'DefaultAxesFontSize', 10);
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');
set(0, 'DefaultColorbarTickLabelInterpreter', 'latex');
set(0, 'defaultAxesLabelFontSizeMultiplier', 1.2);

% Figure setup
set(0, 'defaultFigureColor', 'w');
set(0, 'defaultAxesColor', 'w');
%set(0, 'defaultFigurePosition', [100, 100, 1.1*624/2, 1.1*624*(2/4)/2]);
set(0, 'DefaultAxesClipping', 'on');
set(0, 'DefaultLineClipping', 'on');
set(0, 'defaultAxesLineWidth', 1.5);
set(0, 'defaultLineLineWidth', 2);

% Colors
NTNU_black  = [0,0,0];
NTNU_blue   = [0,80,158]/255;
NTNU_yellow = [247,208,25]/255;
NTNU_orange = [239,129,20]/255;
NTNU_brown  = [207,184,135]/255;
NTNU_purple = [176, 27, 129]/255;
NTNU_green  = [188, 208, 37]/255;

colororder = [NTNU_blue; NTNU_brown; NTNU_brown];

% Plotting
figure(1);
clf; grid on; hold on; box on;

bar([S_CO, S_CO_balanced],'linewidth',1.5, 'BarWidth',1)

set(gca, 'ColorOrder', colororder);
set(gca,'yscale','log');
title('Singular values of controllability matrix')
xlim([0,13])
xticks(1:12)
xlabel('$n$')
ylabel('$\sigma(C)$')
legend({'Normal','Balanced'},'location','best')
xline([4.5, 10.5],'r--','linewidth',3, 'HandleVisibility','off')

figure(2);
clf; grid on; hold on; box on;

bar([S_OB, S_OB_balanced],'linewidth',1.5, 'BarWidth',1)

set(gca, 'ColorOrder', colororder);
set(gca,'yscale','log');
title('Singular values of observability matrix')
xlim([0,13])
xticks(1:12)
xlabel('$n$')
ylabel('$\sigma(O)$')
legend({'Normal','Balanced'},'location','best')
xline([4.5, 10.5],'r--','linewidth',3, 'HandleVisibility','off')