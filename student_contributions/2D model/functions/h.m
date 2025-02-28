function y = h(x, params)
    % h: Compute the magnetic field at a measurement point produced by the levitating magnet.
    %
    %   y = h(x, u, params)
    %
    % Inputs:
    %   x      = [x, z, theta, dx, dz, dtheta] : state of the levitating magnet.
    %   u      = control input (not used in h here).
    %   params = structure containing parameters including:
    %             - mu0: permeability of free space [H/m]
    %             - m_l_nom: nominal dipole moment of the levitating magnet
    %             - p: measurement point (2x1 vector). If not provided, defaults to [0;0].
    %
    % Output:
    %   y = [B_x; B_z] is the magnetic field (in Tesla) at the measurement point.
    
    % Determine measurement point p (default to origin if not provided)
    if isfield(params, 'p')
         p = params.p(:); % ensure p is a column vector
    else
         p = [0; 0];
    end

    % Position of the levitating magnet (x and z components)
    r_l = [x(1); x(2)];
    
    % Vector from magnet to measurement point
    r_vec = p - r_l;
    r_norm = norm(r_vec);
    if r_norm < 1e-6
         r_norm = 1e-6;  % avoid singularity
    end
    
    m_eff = params.m_l_nom*[-sin(x(3)); cos(x(3))];
    
    % Compute the magnetic field using the standard dipole formula
    y = (params.mu0/(4*pi))*(3*(dot(m_eff, r_vec))*r_vec/(r_norm^5) - m_eff/(r_norm^3));
end
