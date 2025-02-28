clc; clear all; 
addpath('.\functions')

%% Define system parameters
parameters;

%% Find equilibrium point
index = @(A,i) A(i);
fz = @(z) index(f([0,z,zeros(1,4)]',[0,0]',params),5);

zeq =  fzero(fz,0.1);

xeq = [0,zeq,zeros(1,4)]';
ueq = [0,0]';

%% Linearize model
xlp = xeq;
ulp = ueq;

[A,B,C] = linearizeModel(@f,@h,xlp,ulp,params);

M = [1,-1]'; % Map to ensure that the solenoids "oppose" each other
B = B*M;

G = tf(ss(A,B,C,[]));
G = G(1,1);

G % Just to display the transfer function

%% Define output feedback PD controller
kp = 300;
kd = 20;

u = @(t,x) -kp*M*[1,0]*h(x,params) - kd*M*[1,0]*dh(x,params); % [1,0] -> Only use the bx measurement (i.e., not bz)

% Check if linearized model is stable
K = tf([kd,kp],1);
M = feedback(G*K,1);

disp(real(eig(M))')

%% Simulation
t = linspace(0,2,500);
x0 = [0.002, zeq+0.02, 0.1, 0, 0, 0]';

[t,x] = ode45(@(t,x) f(x, u(t,x), params), t, x0);

%% Compute input and output
Y = zeros(length(t),2);
U = zeros(length(t),2);
for i = 1:length(t)
    Y(i,:) = h(x(i,:)',params)';
    U(i,:) = u(t(i),x(i,:)')';
end

%% Figures
% Equilibrium curve
figure(1);
clf; hold on; box on;
title('Equilibrium Curve','interpreter','latex','fontsize',14)
xlabel('$z$','interpreter','latex','fontsize',14)
ylabel('$F_z$','interpreter','latex','fontsize',14)

z = linspace(0,0.1,300);
for i = 1:length(z)
    plot(z(i), fz(z(i)),'b.','handlevisibility','off')
end
yline(0,'handlevisibility','off')
xline(zeq,'handlevisibility','off')

plot(zeq,fz(zeq),'rx','LineWidth',2,'MarkerSize',15,'DisplayName','Equilibrium point')
legend('location','best','Interpreter','latex')


% Time series
figure(2);
clf; 

subplot(2,2,1);
hold on; box on;
title('Position of magnet','interpreter','latex','fontsize',12)
xlabel('$t$ [s]','Interpreter','latex','FontSize',12)
ylabel('$x/z$ [m]','Interpreter','latex','FontSize',12)

plot(t, x(:,1:2))

legend({'$x$', '$z$'},'Interpreter','latex')


subplot(2,2,2);
hold on; box on;
title('Angle of magnet','interpreter','latex','fontsize',12)
xlabel('$t$ [s]','Interpreter','latex','FontSize',12)
ylabel('$\theta$ [rad] ','Interpreter','latex','FontSize',12)

plot(t, x(:,3))


subplot(2,2,3);
hold on; box on;
title('Measurements','interpreter','latex','fontsize',12)
xlabel('$t$ [s]','Interpreter','latex','FontSize',12)
ylabel('$B_{x/z}$ [T]','Interpreter','latex','FontSize',12)

plot(t, Y)

legend({'$B_x$', '$B_z$'},'Interpreter','latex')


subplot(2,2,4);
hold on; box on;

title('Solenoid currents','interpreter','latex','fontsize',12)
xlabel('$t$ [s]','Interpreter','latex','FontSize',12)
ylabel('$I$ [A]','Interpreter','latex','FontSize',12)

yline(1.4*[-1,1],'--')
plot(t, U)

legend({'Constraints','','$I^+$', '$I^-$'},'Interpreter','latex')