% Mechanical
params.M   = 0.075;         % Mass of levitating magnet [kg]
params.g   = 9.81;          % Gravitational acceleration [m/s^2]

% Magnetic (using remanence as the material property)
params.mu0   = 4*pi*1e-7;   % Permeability of free space [H/m]
params.Br_p  = 1;           % Remanence of "positive" magnet [T]
params.Br_n  = 1;           % Remanence of "negative" magnet [T]
params.Br_l  = 1.2;         % Remanence of levitating magnet [T]

% Geometric parameters for cylindrical magnets
params.r_base = 0.005;      % Radius of base magnets [m]
params.h_base = 0.01;       % Height of base magnets [m]
params.r_lev  = 0.05/2;     % Radius of levitating magnet [m]
params.h_lev  = 0.005;      % Height of levitating magnet [m]

% Compute dipole moments from remanence and geometry
params.m_p_nom = (params.Br_p/params.mu0)*(pi*params.r_base^2*params.h_base);
params.m_n_nom = (params.Br_n/params.mu0)*(pi*params.r_base^2*params.h_base);
params.m_l_nom = (params.Br_l/params.mu0)*(pi*params.r_lev^2 *params.h_lev);

% Compute moment of inertia for the levitating magnet (a cylindrical disk) about a horizontal axis
params.J = (1/4)*params.M*(params.r_lev)^2 + (1/12)*params.M*(params.h_lev)^2;

% Positions of base magnets
params.rp  = 0.023*[1, 0];   % Position of "positive" magnet
params.rn  = 0.023*[-1, 0];  % Position of "negative" magnet
params.nw  = 480;           % Number of turns in a solenoid (if needed)