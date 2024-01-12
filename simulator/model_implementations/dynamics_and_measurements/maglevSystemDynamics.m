function dx = maglevSystemDynamics(x,u,params,modelName)
%{
Computes the derivative of the state for a magnetic levitation system.
- x: current state
- u: control input
- params: system parameters
%}

% Computing force and torque on levitating magnet
[fx,fy,fz,tx,ty,tz] = computeForceAndTorque(x,u,params,modelName);

% Setting up system matrices
A = [
    zeros(6), eye(6);
    zeros(6), zeros(6)
    ];

B = [
    zeros(6);
    eye(6)
    ];

% Mass and inertia properties of the magnet
M = [
    params.magnet.m*eye(3), zeros(3);
    zeros(3), diag(params.magnet.I)
    ];

% Computing the nonlinear function f(x,u)
f = M\([fx;fy;fz;tx;ty;tz]-[zeros(3,1);cross(x(10:12),diag(params.magnet.I)*x(10:12))])...
    -[zeros(2,1);params.physical.g;zeros(3,1)];

dx = A*x+B*f;
