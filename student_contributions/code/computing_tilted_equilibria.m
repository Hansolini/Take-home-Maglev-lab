% Load parameters
parameters_maggy_V2;

% Adjusting parameters (to speed up computation)
params.solenoids.r = [];
params.solenoids.x = [];
params.solenoids.y = [];
params.solenoids.z = [];

% Setup system equations
modelName = 'fast';
f = @(x,u) maglevSystemDynamics(x,u,params,modelName);

alpha = 0;
beta  = 0;
gamma = 0;

% Define force function
index = @(v,i) v(i);
force = @(x,y,z,beta) index(f([x,y,z,alpha,beta,gamma,0,0,0,0,0,0]',zeros(length(params.solenoids.r),1)),7:9);

% % Define grid points
% n = 10;
% [X,Z] = meshgrid(linspace(-0.04,0.04,n), linspace(1.5*max(params.permanent.l),0.06,n));
% 
% % Compute force
% FX   = zeros(size(X));
% FZ   = zeros(size(X));
% NORM = zeros(size(X));
% tic
% for i = 1:size(X,1)
%     for j = 1:size(X,2)
%         temp = force(X(i,j),0,Z(i,j),beta);
%         FX(i,j) = temp(1);
%         FZ(i,j) = temp(3);
%         NORM(i,j) = norm([FX(i,j),FZ(i,j)]);
%     end
% end
% toc

%% Figure
figure(1);
clf; grid on; hold on; axis equal;

%surf(X,Z,1*(NORM < 1e0))

%xlim([min(X(:)), max(X(:))])
%ylim([min(Z(:)), max(Z(:))])
xlabel('x')
ylabel('z')
shading interp;

% Optimize over x and y
x0 = [0,0,0.036]';

BETA = linspace(0,pi/4,301);

XEq = zeros(size(BETA));
YEq = zeros(size(BETA));
ZEq = zeros(size(BETA));
for i = 1:length(BETA)
    objfun = @(x) norm(force(x(1),x(2),x(3),BETA(i)));
    x = fmincon(objfun,x0,[],[],[],[],[-0.05,0,0],[0.05,0,0.1]);
    x0 = x;

    XEq(i) = x(1);
    YEq(i) = x(2);
    ZEq(i) = x(3);
    
    if objfun(x) < 1e-3
        plot(x(1),x(3),'gx','linewidth',2)
    else
        plot(x(1),x(3),'r.','linewidth',2)
    end
    drawnow;


end