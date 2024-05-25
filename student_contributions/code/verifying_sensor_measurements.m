%% Load the parameters "params"
parameters_maggy_V2;
%% Setup system
modelName = 'accurate';

% Set up the measurement equation
u = [0;0;0;0];
h = @(x) maglevSystemMeasurements(x,u,params,modelName);

%% Compute measurements over a range
[px,py,pz] = meshgrid(linspace(-0.1,0.1,100),0,0.05);

y = zeros(9,length(px));
for i = 1:length(px)
    y(:,i) = h([px(i), py(i), pz(i), zeros(1,9)]');
end

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

plot(px,y(1,:),'b-','displayname','Sensor Center')
%plot(px,y(5,:),'r--','displayname','Sensor Off-center')
%plot(px,y(8,:),'g:','displayname','Sensor Off-center 2')
legend('location','best')
set(gca, 'ColorOrder', colororder);
xlabel('x')
title('Measurements when y = 0 and z = 0.05')
ylabel('Measurements')
