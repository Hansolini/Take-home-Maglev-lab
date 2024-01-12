clc; clear all;
addpath(genpath(pwd));

%% Load the parameters "params"
parameters_maggy_V2;


eta = [0,0.0,0.055,pi/4,0,0]';
R = rot(eta(4),eta(5),eta(6));

x = 0.0;
y = 0;
z = 0.06;

[bxMagnet, byMagnet, bzMagnet] = computeFieldTotal(x,y,z,eta,u,params,modelName);

figure(1);
clf; grid on; hold on; box on; daspect([1,1,1]);
view([45,15]);

xlim([-0.1,0.1])
ylim([-0.1,0.1])
zlim([-0.1,0.1])
plotBaseArtistic(params);

plotMagnetArtistic(eta(1),eta(2),eta(3),eta(4),eta(5),eta(6),params);
plotMagnetArtistic(etaRotated(1),etaRotated(2),etaRotated(3),etaRotated(4),etaRotated(5),etaRotated(6),params);
quiver3(x,y,z,bxMagnet,byMagnet,bzMagnet,'LineWidth',2,'Color','r','AutoScaleFactor',10)

%% Compute radius correction factor
cFast = computeSolenoidRadiusCorrectionFactor(params,'fast');
cAccurate = computeSolenoidRadiusCorrectionFactor(params,'fast');

paramsFast = params;
paramsFast.solenoids.r = cFast*paramsFast.solenoids.r;

paramsAccurate = params;
paramsAccurate.solenoids.r = cAccurate*paramsAccurate.solenoids.r;


%% Compute z curve
% Defining range
zMin = max([params.solenoids.z+params.solenoids.l/2, params.permanent.z+params.permanent.l/2]) + params.magnet.l/2; % Physical Limitations
zMax = 0.1; % Arbitrary
z = linspace(zMin,zMax,100);

paramsCorrected = params;
paramsCorrected.solenoids.r = 0.5584*paramsCorrected.solenoids.r;

% Computing
fz = computeVerticalForce(z,paramsCorrected,'fast');
fza = computeVerticalForce(z,paramsCorrected,'accurate');
fzf = computeVerticalForce(z,params,'fillament');

% Plotting
figure(1);
clf; grid on; hold on;
plot(z,fz,'b','linewidth',2)
plot(z,fza,'r--','linewidth',2)
plot(z,fzf,'k:','linewidth',2)
yline(0,'linewidth',2)

[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(params,'fast');

try
    xline(zEq)
catch
end

try
    xline(zEqInv)
catch
end

%% Compute magnetic field

nr = 4*5;
nz = 100;

theta = linspace(0,2*pi - 2*pi/nr,nr);

x = params.magnet.r*cos(theta);
y = params.magnet.r*sin(theta);
z = linspace(params.magnet.l/2+max(max(params.solenoids.z + params.solenoids.l/2), max(params.permanent.z + params.permanent.l/2)),0.1,nz);

X = repmat(x,nz,1);
Y = repmat(y,nz,1);
Z = repmat(z',1,nr);

u = 100*ones(4,1);

[bx,by,bz] = computeFieldBase(X,Y,Z,u,params,'accurate');

fx = zeros(1,nz);
fy = zeros(1,nz);
fz = zeros(1,nz);

for i = 1:nz
    [fxTemp,fyTemp,fzTemp] = computeForceAndTorque([0;0;z(i);zeros(9,1)],u,params,'accurate');

    fx(i) = fxTemp;
    fy(i) = fyTemp;
    fz(i) = fzTemp;
end

NORM = sqrt(fx.^2 + fy.^2 + fz.^2);

%fx = fx./NORM;
%fy = fy./NORM;
%fz = fz./NORM;

NORM = sqrt(bx.^2 + by.^2 + bz.^2);

%bx = bx./NORM;
%by = by./NORM;
%bz = bz./NORM;

figure(2);
clf; grid on; hold on; box on; axis equal; %view([45,15]); 
plotBaseArtistic(params);
quiver3(X,Y,Z,bx,by,bz,'b')
quiver3(zeros(size(z)),zeros(size(z)),z,fx,fy,fz,'r')
plot3(x,y,0.09*ones(size(x)),'r.','MarkerSize',15)

figure(3);
clf; grid on; hold on;
plot(z,fx,'linewidth',2)
plot(z,fy,'linewidth',2)
%plot(z,fz,'linewidth',2)


%% Load other parameters
figure(1);
clf; grid on; hold on; box on; daspect([1,1,1]);
view([45,15]);

xlim([-0.1,0.1])
ylim([-0.1,0.1])
zlim([-0.1,0.1])
plotBaseArtistic(params);
H = plotMagnetArtistic(0,0,0.05,0,0.5,0.5,params);

%% Figure
modelName = 'fast';

% Set up the system equation
f = @(x,u) maglevSystemDynamics(x,u,params,modelName);

% Set up the measurement equation
h = @(x,u) maglevSystemMeasurements(x,u,params,modelName);

% Find equilibrium
index = @(A,i) A(i);
foo = @(z) index(f([0,0,z,zeros(1,9)]',zeros(4,1)),9);

z_sample_points = linspace(0,0.06,1000);
dz = zeros(size(z_sample_points));
for i = 1:length(z_sample_points)
    dz(i) = foo(z_sample_points(i));
end

[~,I] = min(abs(dz(1:end-1))+sign(abs(diff(dz))));
zEq = z_sample_points(I);

% Linearization
xLp = [0,0,zEq,zeros(1,9)]';
uLp = zeros(length(params.solenoids.r),1);

delta = 1e-6;
[A,B,C,D] = finiteDifferenceLinearization(f,h,xLp,uLp,delta);


xdot_lp = f(xLp, uLp);
y_lp = h(xLp,uLp);

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

% Simulation setup
x0 = xLp+[0.001 -0.003 0.04 pi/10 0 0 zeros(1, 6)].';
t_span = linspace(0,1,100);

disp('Simulation...')
tic;

[t,x] = ode15s(@(t,x) f(x,-K*(x-xLp)-uLp), t_span, x0);

%%
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


%%
clc;
I = [1,2,3,4,5,7,8,9,10,11]';
Ared = A(I,I);
Bred = B(I,:);
Cred = C(:,I);
Dred = D;

lambda = -100*(10:-1:1)';

Lred = place(Ared',Cred',lambda)';
Mred = Ared-Lred*Cred;

disp(cond(Lred))
disp(sort(eig(Mred)'))
disp(sort(lambda'))

u = -K*(x'-xLp)-uLp;
y = zeros(6,length(t));
for i = 1:length(t)
    y(:,i) = h(x(i,:)',u(:,i));
end

yLp = h(xLp,uLp);

fy = @(T) interp1(t',y',T)';
fu = @(T) interp1(t',u',T)';

z0 = rand(10,1);
[~,z] = ode45(@(t,z) Ared*z + Bred*fu(t) + Lred*((fy(t) - yLp) - Cred*z - Dred*fu(t)), t, z0);
z = z + xLp(I)';

figure(1);
clf; 

subplot(2,1,1)
grid on; hold on;
plot(t,x(:,1:5),'linewidth',2)
plot(t,z(:,1:5),'--','linewidth',2)

ylim([-0.01,0.1])

subplot(2,1,2)
grid on; hold on;

plot(t,fy(t) - yLp,'-','linewidth',2)
%plot(t,fu(t),'--','linewidth',2)