function params = parameters()
    % ------------------------------
    % Mechanical Properties
    % ------------------------------
    params.M_lev = 0.075;        % Mass of levitating magnet [kg]
    params.g     = 9.81;         % Gravitational acceleration [m/s^2]
    
    % ------------------------------
    % Magnetic Properties
    % ------------------------------
    params.mu0 = 4*pi*1e-7;      % Permeability of free space [H/m]
    
    % Remanence [T]
    Br_base = 1.0;               % Base magnets
    Br_lev  = 1.2;               % Levitating magnet
    
    % Base magnet geometry
    params.r_base = 0.005;       % Radius [m]
    params.h_base = 0.010;       % Height [m]
    
    % Levitating magnet geometry
    params.r_lev = 0.025;        % Radius [m]
    params.h_lev = 0.005;        % Height [m]
    
    % Dipole moments (scalar magnitudes)
    V_base = pi*params.r_base^2*params.h_base;
    V_lev  = pi*params.r_lev^2*params.h_lev;
    
    params.m_base = (Br_base/params.mu0)*V_base;
    params.m_lev  = (Br_lev/params.mu0)*V_lev;
    
    % ------------------------------
    % Inertia (Solid Cylinder)
    % ------------------------------
    params.I_lev = eye(2)*(1/4)*params.M_lev*params.r_lev^2 + (1/12)*params.M_lev*params.h_lev^2;
    
    % ------------------------------
    % Base Magnets and Sensors
    % ------------------------------
    params.r_base_array = 0.023*[  
        1, -1,  0,  0;
        0,  0,  1, -1;
        0,  0,  0,  0 ];
    
    params.r_sensor_array = 0.01*[  
        0,   1,   0;
        0,   0,   1;
        0,   0,   0];
    
    % ------------------------------
    % Actuation Geometry
    % ------------------------------
    params.n_w = 480;            % Number of turns per solenoid
    
    % ------------------------------
    % Dimensions
    % ------------------------------
    params.n_u = size(params.r_base_array, 2);  % Number of base magnets
    params.n_y = 3*size(params.r_sensor_array, 2);  % Number of sensors
end
