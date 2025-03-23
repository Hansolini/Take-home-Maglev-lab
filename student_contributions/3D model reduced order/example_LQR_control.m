clc; clear all;
addpath(genpath('.\functions'))

%% Define system parameters
params = parameters;

%% Find equilibrium point
index = @(A,i) A(i);
fz = @(z) index(f([0,0,z,zeros(1,7)]',[0,0,0,0]',params),8);  % state is now 10x1

zeq =  fzero(fz,0.1);

xeq = [0,0,zeq,zeros(1,7)]';
ueq = [0,0,0,0]';

%% Linearize model
xlp = xeq;   % 10x1 equilibrium state
ulp = ueq;

[A,B,C] = linearizeModel(@f, @h, xlp, ulp, params);

%% Define LQR controller
Q = diag([ ...
    1e1,1e1,1e1, ...     % position
    1e1,1e1, ...         % orientation (phi, theta)
    1e1,1e1,1e5, ...     % velocity
    1e1,1e1              % angular velocity
]);
R = 1e2*eye(params.n_u);

K = lqr(ss(A, B, eye(10), []), Q, R);
u = @(t,x) -K*x;

%% Simulation
t = linspace(0, 2, 500);
x0 = [-0.001, 0.001, zeq + 0.02, ...  % position
      pi/10, 0, ...                   % phi, theta
      0, 0, 0, ...                    % velocity
      0, 0]';                         % angular velocity

[t,x] = ode45(@(t,x) f(x, u(t,x), params), t, x0);

%% Compute input and output
Y = zeros(length(t), params.n_y);
U = zeros(length(t), params.n_u);
for i = 1:length(t)
    Y(i,:) = h(x(i,:)', params)';
    U(i,:) = u(t(i), x(i,:)')';
end

%% Figures
% Equilibrium curve
figure(1);
clf; hold on; box on;
title('Equilibrium Curve','interpreter','latex','fontsize',14)
xlabel('$z$','interpreter','latex','fontsize',14)
ylabel('$F_z$','interpreter','latex','fontsize',14)

z = linspace(0, 0.1, 300);
for i = 1:length(z)
    plot(z(i), fz(z(i)),'b.','handlevisibility','off')
end
yline(0,'handlevisibility','off')
xline(zeq,'handlevisibility','off')

plot(zeq, fz(zeq), 'rx','LineWidth',2,'MarkerSize',15,'DisplayName','Equilibrium point')
legend('location','best','Interpreter','latex')


% Time series
figure(2);
clf; 

subplot(2,2,1);
hold on; box on;
title('Position of magnet','interpreter','latex','fontsize',12)
xlabel('$t$ [s]','Interpreter','latex','FontSize',12)
ylabel('$x/y/z$ [m]','Interpreter','latex','FontSize',12)

plot(t, x(:,1:3))
legend({'$x$','$y$', '$z$'},'Interpreter','latex')


subplot(2,2,2);
hold on; box on;
title('Angle of magnet','interpreter','latex','fontsize',12)
xlabel('$t$ [s]','Interpreter','latex','FontSize',12)
ylabel('$\phi$, $\theta$ [rad]','Interpreter','latex','FontSize',12)

plot(t, x(:,4:5))
legend({'$\phi$', '$\theta$'},'Interpreter','latex')


subplot(2,2,3);
hold on; box on;
title('Measurements','interpreter','latex','fontsize',12)
xlabel('$t$ [s]','Interpreter','latex','FontSize',12)
ylabel('$B_{x/y/z}$ [T]','Interpreter','latex','FontSize',12)

plot(t, Y(:,1:3))
legend({'$B_x$', '$B_y$', '$B_z$'},'Interpreter','latex')


subplot(2,2,4);
hold on; box on;
title('Solenoid currents','interpreter','latex','fontsize',12)
xlabel('$t$ [s]','Interpreter','latex','FontSize',12)
ylabel('$I$ [A]','Interpreter','latex','FontSize',12)

yline(1.4*[-1,1],'--')
plot(t, U)
legend({'Constraints',''},'Interpreter','latex')
