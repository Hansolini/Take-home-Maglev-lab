function T = rotRateEuler(eta)
%ROTRATEEULER Euler angle rate matrix using only roll (phi) and pitch (theta)
%   omega = T * [dot(phi); dot(theta)]

    phi = eta(1);
    % theta = eta(2); % Can be ignored or used for extensions

    T = [1,        0;
         0,  cos(phi)];
end
