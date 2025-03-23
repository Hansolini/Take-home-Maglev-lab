function F_i = force(m_i, m, r)
%FORCE Calculates force on dipole 2 due to dipole 1
%   Inputs:
%       m_i - 3x1 magnetic moment of magnet i
%       m   - 3x1 magnetic moment of levitating magnet
%       r   - 3x1 vector from magnet i to levitating magnet
%   Output:
%       F_i - 3x1 force vector on levitating magnet

    mu0 = 4*pi*1e-7;  % Vacuum permeability
    r_norm = norm(r);

    term1 = (m_i'*r)*m;
    term2 = (m'*r)*m_i;
    term3 = (m_i'*m)*r;
    term4 = 5*(m_i'*r)*(m'*r)/r_norm^2*r;

    F_i = (3*mu0/(4*pi*r_norm^5))*(term1 + term2 + term3 - term4);
end
