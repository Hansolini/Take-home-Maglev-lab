function [A,B,C] = linearizeModel(f,h,xlp,ulp,params)
    delta = 1e-10;
    
    A = zeros(6, 6);    
    for i = 1:6
        xp = xlp;
        xn = xlp;
    
        xp(i) = xp(i) + delta;
        xn(i) = xn(i) - delta;
        
        dxp = f(xp, ulp, params);
        dxn = f(xn, ulp, params);
        
        A(:, i) = (dxp - dxn)/(2*delta);
    end
    
    B = zeros(6, 2);    
    for i = 1:2
        up = ulp;
        un = ulp;
    
        up(i) = up(i) + delta;
        un(i) = un(i) - delta;
        
        dxp = f(xlp, up, params);
        dxn = f(xlp, un, params);
        
        B(:, i) = (dxp - dxn)/(2*delta);
    end
    
    C = zeros(2, 6);    
    for i = 1:6
        xp = xlp;
        xn = xlp;
    
        xp(i) = xp(i) + delta;
        xn(i) = xn(i) - delta;
        
        yp = h(xp, params);
        yn = h(xn, params);
        
        C(:, i) = (yp - yn)/(2*delta);
    end
end
