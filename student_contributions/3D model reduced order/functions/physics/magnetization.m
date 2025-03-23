function M = magnetization(phi, theta, psi, m)
%MAGNETIZATIONFROMEULER Returns magnetization vector in world frame
%   Inputs:
%       phi   - yaw (rotation around z-axis)
%       theta - pitch (rotation around y-axis)
%       psi   - roll (rotation around x-axis)
%       m     - magnitude of magnetic moment
%
%   Output:
%       m_world - 3x1 vector, magnetization in world coordinates

    % Rotation matrices (ZYX convention)
    Rz = [cos(phi), -sin(phi), 0;
          sin(phi),  cos(phi), 0;
               0,         0, 1];
    
    Ry = [cos(theta), 0, sin(theta);
                0, 1,        0;
         -sin(theta), 0, cos(theta)];
    
    Rx = [1,       0,        0;
          0, cos(psi), -sin(psi);
          0, sin(psi),  cos(psi)];
    
    % Full rotation matrix
    R = Rz*Ry*Rx;

    % Rotate to world frame
    M = R*[0; 0; m];
end
