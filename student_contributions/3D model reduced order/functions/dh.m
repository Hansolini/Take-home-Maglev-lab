function ydot = dh(x, params)
    % dh: Compute the time derivative of the magnetic field measurement.
    %
    %   ydot = dh(x, u, params)
    %
    % This function numerically estimates the Jacobian of h with respect to the 
    % state [x; z; theta] using finite differences, and then multiplies it by the 
    % velocity part [dx; dz; dtheta] to yield the time derivative.
    
    delta = 1e-10;

    J = zeros(params.n_y, 3);    
    for i = 1:3
        xp = x;
        xn = x;

        xp(i) = xp(i) + delta;
        xn(i) = xn(i) - delta;
        
        yp = h(xp, params);
        yn = h(xn, params);
        
        J(:, i) = (yp - yn)/(2*delta);
    end
    
    % The time derivative of [x; z; theta] is given by [dx; dz; dtheta] = x(4:6)
    xdot_pos = x(4:6);
    ydot = J*xdot_pos;
end
