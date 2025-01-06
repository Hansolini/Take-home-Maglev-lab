%% Load the parameters "params"
parameters_maggy_V2;

%% Setup system
modelName = 'fillament';

% Set up the measurement equation
h = @(x,u) maglevSystemMeasurements(x,u,params,modelName);

%% Optimization
U = linspace(-0.52,0.52,100);
BX = zeros(size(U));

for i = 1:length(U)
    u = [U(i),0,-U(i),0];
    b = h(1e10*ones(12,1),u);
    BX(i) = b(1);
end

% Plot results of test
u = -255:255;
I = sharedData.BX ~= 0;

figure(2);
clf; grid on; hold on; box on;
plot(u(I),(sharedData.BX(I) - sharedData.BX(I(round(end/2))))/1000,'b','LineWidth',2,'DisplayName','Measured')
plot(255*U/max(U),BX,'r--','LineWidth',2,'DisplayName','Simulated')
xticks(255*U(1:5:end)/max(U(1:5:end)))
xticklabels(round(U(1:5:end),2))
legend('location','best')
title('Comparing real measurements to simulator')
ylabel('$B_x$ [T]')
xlabel('Current [A]')

xline([-150,150],'k--','linewidth',2,'HandleVisibility','off')
