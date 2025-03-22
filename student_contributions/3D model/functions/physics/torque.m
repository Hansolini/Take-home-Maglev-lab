function tau_i = torque(m_i, m, r)
%TORQUE Calculates torque on levitating magnet due to base magnet i
%   Inputs:
%       m_i - 3x1 magnetic moment of base magnet i
%       m   - 3x1 magnetic moment of levitating magnet
%       r   - 3x1 vector from magnet i to levitating magnet (r - r_i)
%   Output:
%       tau_i - 3x1 torque vector on levitating magnet

    mu0 = 4*pi*1e-7;  % Vacuum permeability

    m_i = m_i(:); m = m(:); r = r(:);

    r_norm = norm(r);
    r_hat = r/r_norm;

    % Magnetic field at position of levitating magnet due to magnet i
    B = (mu0/(4*pi*r_norm^3))*(3*dot(m_i, r_hat)*r_hat - m_i);

    % Torque on levitating magnet
    tau_i = cross(m, B);
end
