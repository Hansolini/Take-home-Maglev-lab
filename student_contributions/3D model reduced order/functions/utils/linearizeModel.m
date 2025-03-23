function [A,B,C] = linearizeModel(f,h,xlp,ulp,params)
    delta = 1e-10;
    nx = length(xlp);
    nu = params.n_u;
    ny = params.n_y;

    A = zeros(nx);    
    for i = 1:nx
        xp = xlp;
        xn = xlp;
    
        xp(i) = xp(i) + delta;
        xn(i) = xn(i) - delta;
        
        dxp = f(xp, ulp, params);
        dxn = f(xn, ulp, params);
        
        A(:, i) = (dxp - dxn)/(2*delta);
    end
    
    B = zeros(nx, nu);    
    for i = 1:nu
        up = ulp;
        un = ulp;
    
        up(i) = up(i) + delta;
        un(i) = un(i) - delta;
        
        dxp = f(xlp, up, params);
        dxn = f(xlp, un, params);
        
        B(:, i) = (dxp - dxn)/(2*delta);
    end
    
    C = zeros(ny, nx);    
    for i = 1:nx
        xp = xlp;
        xn = xlp;
    
        xp(i) = xp(i) + delta;
        xn(i) = xn(i) - delta;
        
        yp = h(xp, params);
        yn = h(xn, params);
        
        C(:, i) = (yp - yn)/(2*delta);
    end
end
