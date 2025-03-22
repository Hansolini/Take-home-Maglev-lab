function T = rotRateEuler(eta)
%ROTRATEEULER Euler angle rate transformation matrix (ZYX convention)

    phi = eta(1);
    theta = eta(2);

    T = [1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
         0, cos(phi),            -sin(phi);
         0, sin(phi)/cos(theta),  cos(phi)/cos(theta)];
end
