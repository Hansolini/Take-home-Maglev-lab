function R = rotEuler(eta)
%ROTEULER Rotation matrix using only roll (phi) and pitch (theta)
%   eta = [phi; theta]

    phi = eta(1);
    theta = eta(2);

    R = [ cos(theta),        sin(theta)*sin(phi),  sin(theta)*cos(phi);
          0,                 cos(phi),            -sin(phi);
         -sin(theta),        cos(theta)*sin(phi),  cos(theta)*cos(phi) ];
end
