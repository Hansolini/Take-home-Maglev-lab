load("Dataset.mat");
parameters_maggy_V2;

% x0=xLp+[0.0025;0;0.0025;zeros(9,1)];  %Initial condition for simulink
A_n = [                 %System matrices
    zeros(6), eye(6);
    zeros(6), zeros(6)
    ];

B_n = [
    zeros(6);
    eye(6)
    ];

poles = [-100,-300,-100,-120,-300,-320,-170,-270,-500,-230, -90, -140];
K_tst = place(A_n,B_n,poles);  %For full-state feedback

%% Noise and filter coefficient
fs = 1000;
sigma = 65e-6;
%% Matrices for only x,y,z,alpha,beta,gamma controller

A_xyz = A_n([1:5,7:11],[1:5,7:11]);
B_xyz = B_n([1:5,7:11],:);
K_xyz = place(A_xyz,B_xyz,[-0.5,-0.5,-0.5,-40,-30,-2000,-2000,-9000,-25,-20]); %Replace these poles if you want to modify system behaviour

I_m = [params.permanent.J/params.physical.mu0*params.permanent.l(1);
    params.permanent.J/params.physical.mu0*params.permanent.l(2);
    params.permanent.J/params.physical.mu0*params.permanent.l(3);
    params.permanent.J/params.physical.mu0*params.permanent.l(4);
    ];

