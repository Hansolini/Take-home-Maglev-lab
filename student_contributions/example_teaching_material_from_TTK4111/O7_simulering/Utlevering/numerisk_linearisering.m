%% Oppsett simulator
parameters;
f = @(x,u) maglevSystemDynamics2d(x,u,params);
h = @(x,u) maglevSystemMeasurements2d(x,params);

%% Oppsett lineær modell
% Parametre
R = 0.03; % avstand fra origo for hybridmagneter

rl = params.magnet.r; % radius svevemagnet
ll = params.magnet.l; % lengde av svevemagnet

rp = params.permanent.r(1); % radius permanentmagneter
lp = params.permanent.l(1); % lengde av permanentmagneter

rs = params.solenoids.r(1); % radius solenoider
nw = params.solenoids.nw; % antall viklinger solenoider

M = params.magnet.m; % Masse svevende magnet
J = (M/12)*(3*rl^2+ll^2); % Treghetsmoment svevende magnet

g = params.physical.g;
mu0 = params.physical.mu0;

ml = abs(params.magnet.J)*pi*rl^2*ll/mu0; % magnetisk moment svevemagnet
m = abs(params.permanent.J)*pi*rp^2*lp/mu0; % magnetisk moment permanent magneter

As = pi*rs^2; % areal tverrsnitt solenoider

% Likevektspunkt
z_eq = 0.0365;

% Koeffisienter for lineær modell
q  = R^2 + z_eq^2;

p1 = -(4*R^4 - 27*R^2*z_eq^2 + 4*z_eq^4)/2;
p2 = -(z_eq*(11*R^4 - 23*R^2*z_eq^2 + z_eq^4))/4;

p3 = -10*(R*(R^2 - 4*z_eq^2))/2;

p4 = 2*(-3*R^4 + 24*R^2*z_eq^2 - 8*z_eq^4);

p5 = 1e3*(z_eq*(3*R^2 - 2*z_eq^2))/2;

p6 = 20*(z_eq*(4*R^2 - z_eq^2));
p7 = 20*(-R^4 + 13*R^2*z_eq^2 - z_eq^4);

p8 = 200*(R*z_eq)/2;

%% Numerisk linearisering
delta = 1e-5;              % Perturbasjon fra lineariseringspunktet
x_eq = [0,z_eq,0,0,0,0]';
u_eq = [0,0]';

% Merk: Tilstandsvektoren til systemet er [x,z,theta, dxdt, dzdt, dthetadt]

% Bruk funksjonen finiteDifferenceLinearization(...) (ligger i den vedlagte
% simulatoren) for å gjøre lineariseringen.
% 
% HUSK å legge til simulatormappen i path: høyreklikk på "simulator" -> 
% "add to path" -> "selected folders and subfolders"

%% Helper Functions
function dx = maglevSystemDynamics2d(x,u,params)
    % ### HACK from 2d to 3d ###
    x_full = [
        x(1),0,x(2),0,x(3),0,...
        x(4),0,x(5),0,x(6),0
        ]';
    u_full = [u(1), u(2), 0, 0]';
    % ##########################
    
    dx = maglevSystemDynamics(x_full,u_full,params);
    
    % ### HACK from 3d to 2d ###
    dx = dx([1,3,5,7,9,11]');
    % ##########################
end

function y = maglevSystemMeasurements2d(x,params)
    % ### HACK from 2d to 3d ###
    x_full = [
        x(1),0,x(2),0,x(3),0,...
        x(4),0,x(5),0,x(6),0
        ]';
    u_full = [0, 0, 0, 0]';
    % ##########################
    
    y = maglevSystemMeasurements(x_full,u_full,params);
    
    % ### HACK from 3d to 2d ###
    y = y([1,3]');
    % ##########################
end