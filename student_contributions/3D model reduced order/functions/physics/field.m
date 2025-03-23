function B_j = field(m, r_j, r)
%FIELD Calculates magnetic field at sensor j from levitating magnet
%   Inputs:
%       m   - 3x1 magnetic moment of levitating magnet
%       r_j - 3x1 position of sensor j
%       r   - 3x1 position of levitating magnet
%   Output:
%       B_j - 3x1 magnetic field at sensor j [T]

    mu0 = 4*pi*1e-7;  % Vacuum permeability

    m = m(:); r_j = r_j(:); r = r(:);
    
    d_j = r_j - r;
    d_norm = norm(d_j);
    d_hat = d_j/d_norm;

    B_j = (mu0/(4*pi*d_norm^3))*(3*dot(m, d_hat)*d_hat - m);
end
