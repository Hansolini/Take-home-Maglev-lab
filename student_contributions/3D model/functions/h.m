function y = h(x, params)
%H Measurement function: magnetic field at sensor positions
%
%   Inputs:
%     x      - 12x1 state vector [r; eta; v; omega]
%     params - structure containing:
%              mu0   - permeability of free space
%              m_lev - scalar magnetic moment of levitating magnet
%              r_j   - [3 x n_y] sensor positions
%
%   Output:
%     y - [3*n_y x 1] stacked magnetic field vectors at all sensors

    % Unpack state
    r     = x(1:3);     % Position of levitating magnet
    eta   = x(4:6);     % Orientation (Euler angles)

    % Unpack parameters
    mu0   = params.mu0;
    m     = params.m_lev;
    r_j   = params.r_sensor_array;
    n_y   = params.n_y;

    % Magnetic moment in world frame
    R = rotEuler(eta);
    m_vec = R*[0; 0; m];

    % Allocate output
    y = zeros(n_y, 1);

    for j = 1:n_y/3
        d = r_j(:,j) - r;         % Vector from magnet to sensor
        d_norm = norm(d);
        if d_norm < 1e-6
            d_norm = 1e-6;        % Avoid singularity
        end

        B = (mu0/(4*pi))*(3*(dot(m_vec,d))*d/d_norm^5 - m_vec/d_norm^3);
        y(3*(j-1)+1:3*j) = B;
    end
end
