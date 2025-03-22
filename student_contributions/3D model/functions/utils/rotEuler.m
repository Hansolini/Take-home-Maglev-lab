function R = rotEuler(eta)
%ROTEULER Rotation matrix from ZYX Euler angles [phi; theta; psi]

    phi = eta(1);
    theta = eta(2);
    psi = eta(3);

    R = [ cos(psi)*cos(theta),  cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi),  cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi);
          sin(psi)*cos(theta),  sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi),  sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
         -sin(theta),           cos(theta)*sin(phi),                               cos(theta)*cos(phi)];
end
