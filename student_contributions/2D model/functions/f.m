function dx = f(x, u, params)
    % x = [x, z, theta, dx, dz, dtheta]
    % u = [du_mp, du_mn] are control inputs that modify the base magnet effective moments.
    %
    % params includes:
    %   rp: [x_p, z_p] position of the positive base magnet
    %   rn: [x_n, z_n] position of the negative base magnet
    %   m_p_nom, m_n_nom: nominal dipole moments of the base magnets (computed from remanence and geometry)
    %   m_l_nom: nominal dipole moment of the levitating magnet (computed from remanence and geometry)
    %   M: mass of the levitating magnet [kg]
    %   J: moment of inertia of the levitating magnet [kg·m^2]
    %   g: gravitational acceleration [m/s^2]
    %   mu0: permeability of free space [H/m]
    
    n = 6;
    dx = zeros(n, 1);
    
    % Base magnet parameters (positions and nominal moments)
    rp = params.rp;  % [x_p, z_p]
    rn = params.rn;  % [x_n, z_n]

    % Modify base magnet moments with control inputs (scaled to represent current in amperes)
    mp = params.m_p_nom + params.nw*pi*params.r_base^2*u(1);
    mn = params.m_n_nom + params.nw*pi*params.r_base^2*u(2);
    
    % Helper function
    R = @(r) ((x(1) - r(1))^2 + (x(2) - r(2))^2);
    
    % First-order derivatives of scalar potential
    dpsi_dx = @(m, r) -3*m*((x(1)-r(1))*(x(2)-r(2)))/(4*pi*(R(r)^(5/2)));
    dpsi_dz = @(m, r)    m/(4*pi)*(1/(R(r)^(3/2)) - 3*(x(2)-r(2))^2/(R(r)^(5/2)));
    
    % Second-order derivaive of scalar potential
    d2psi_dx2  = @(m, r) -3*m*(x(2)-r(2))/(4*pi)*(1/(R(r))^(5/2)  - 5*(x(1)-r(1))^2/(R(r)^(7/2)));
    d2psi_dxdz = @(m, r) -3*m*(x(1)-r(1))/(4*pi)*(1/(R(r))^(5/2)  - 5*(x(2)-r(2))^2/(R(r)^(7/2)));
    d2psi_dz2  = @(m, r) -3*m*(x(2)-r(2))/(4*pi)*(3/(R(r))^(5/2)  - 5*(x(2)-r(2))^2/(R(r)^(7/2)));
    
    % Sum contributions from both base magnets
    tot_d2psi_dx2  = d2psi_dx2(mp, rp)  + d2psi_dx2(mn, rn);
    tot_d2psi_dxdz = d2psi_dxdz(mp, rp) + d2psi_dxdz(mn, rn);
    tot_d2psi_dz2  = d2psi_dz2(mp, rp)  + d2psi_dz2(mn, rn);
    tot_dpsi_dx    = dpsi_dx(mp, rp)    + dpsi_dx(mn, rn);
    tot_dpsi_dz    = dpsi_dz(mp, rp)    + dpsi_dz(mn, rn);
    
    % Compute forces and torque
    F_x = params.mu0*params.m_l_nom*(sin(x(3))*tot_d2psi_dx2  - cos(x(3))*tot_d2psi_dxdz);
    F_z = params.mu0*params.m_l_nom*(sin(x(3))*tot_d2psi_dxdz - cos(x(3))*tot_d2psi_dz2);
    tau = params.mu0*params.m_l_nom*(sin(x(3))*tot_dpsi_dz    + cos(x(3))*tot_dpsi_dx);

    % Dynamic equations
    dx(1) = x(4);
    dx(2) = x(5);
    dx(3) = x(6);
    dx(4) = F_x/params.M;
    dx(5) = F_z/params.M - params.g - 5*x(5);  % Final term HACK to introduce additional damping in z!
    dx(6) = tau/params.J;
end
