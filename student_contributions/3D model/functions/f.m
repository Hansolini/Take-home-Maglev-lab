function dx = f(x, u, params)
%F State-space dynamics of levitating magnet with actuation
%   x      - 12x1 state vector [r; eta; v; omega]
%   u      - [n_u x 1] vector of currents to each solenoid
%   params - struct with fields (see below)
%
    % Unpack state
    r     = x(1:3);
    eta   = x(4:6);
    v     = x(7:9);
    omega = x(10:12);

    % Unpack parameters
    m_lev     = params.m_lev;
    M_lev     = params.M_lev;
    I_lev     = params.I_lev;
    g         = params.g;
    mu0       = params.mu0;
    r_i       = params.r_base_array;
    m_base    = params.m_base;
    r_base    = params.r_base;
    n_w       = params.n_w;
    n_u       = params.n_u;

    % Rotation and transformation matrices
    R = rotEuler(eta);        % World-from-body rotation
    T = rotRateEuler(eta);    % Angular velocity to Euler rates

    % Magnetic moment of levitating magnet in world frame
    m_vec = R*[0; 0; m_lev];

    % Gravitational acceleration
    g_vec = [0; 0; -g];

    % Gain per ampere of current in solenoid
    k = n_w*pi*r_base^2;  % Units: A·m² / A = m²

    % Total force and torque
    F = zeros(3,1);
    tau = zeros(3,1);
    for i = 1:n_u
        m_i = [0; 0; m_base + k*u(i)];
        d_i = r - r_i(:,i);
        F = F + force(m_i, m_vec, d_i);
        tau = tau + torque(m_i, m_vec, d_i);
    end

    % Remove effect of unobservable and uncontrollable states
    % omega(3) = 0;

    % Time derivatives
    dr     = v;
    deta   = T\omega;
    dv     = F/M_lev + g_vec + diag([0,0,-5])*v;  % Final term HACK to introduce additional damping in z!
    domega = I_lev\(tau - cross(omega, I_lev*omega));

    dx = [dr; deta; dv; domega];
end
