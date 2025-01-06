function [X,Y,Z] = cyl(R,N)
    %[X,Y,Z] = cylinder(R,N);
    
    theta = linspace(0,2*pi,N+1);
    
    X = R*cos(theta);
    X = [X;X];
    
    Y = R*sin(theta);
    Y = [Y;Y];
    
    Z = [zeros(1,length(theta));
          ones(1,length(theta))];
end