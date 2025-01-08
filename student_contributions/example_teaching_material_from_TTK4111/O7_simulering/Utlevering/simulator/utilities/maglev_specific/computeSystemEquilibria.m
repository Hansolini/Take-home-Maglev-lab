function [zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(params,modelName)
    
    % Interval limits
    zMin = max([params.solenoids.z+params.solenoids.l/2, params.permanent.z+params.permanent.l/2]) + params.magnet.l/2; % Lower limit is determined by the height of the components
    zMax = 0.5; % Arbitrarily setting half a meter as max limit

    % Divide search space into intervals
    n = 20;
    zRange = linspace(zMin,zMax,n);

    % Objective functions
    objfun = @(z) computeVerticalForce(z,params);

    paramsInv = params;
    paramsInv.magnet.J = -paramsInv.magnet.J;
    objfunInv = @(z) computeVerticalForce(z,paramsInv);

    % Searching for zeros
    zEq = [];
    for i = 1:length(zRange)-1
        try
            zEq(end+1) = fzero(objfun,[zRange(i),zRange(i+1)]);
        catch
            continue;
        end
    end
    zEq = unique(zEq);

    % Searching for inverted zeros
    zEqInv = [];
    for i = 1:length(zRange)-1
        try
            zEqInv(end+1) = fzero(objfunInv,[zRange(i),zRange(i+1)]);
        catch
            continue;
        end
    end
    zEqInv = unique(zEqInv);


    % Computing derivative at each equlibria
    delta = 1e-6;
    
    dzEq = zeros(size(zEq));
    for i = 1:length(dzEq)
        dzEq(i) = (objfun(zEq(i) + delta) - objfun(zEq(i) - delta))/(2*delta);
    end

    dzEqInv = zeros(size(zEqInv));
    for i = 1:length(dzEqInv)
        dzEqInv(i) = (objfunInv(zEqInv(i) + delta) - objfunInv(zEqInv(i) - delta))/(2*delta);
    end

    % Helping function
    function fz = computeVerticalForce(z,params)
        u = zeros(length(params.solenoids.r),1);
        
        % Compute normal graf
        fz = zeros(size(z));
        for j = 1:length(z)
            x = [0;0;z(j);zeros(9,1)];
            [~,~,fz] = computeForceAndTorque(x,u,params,modelName);
        end
        fz = fz/2 - 9.81*params.magnet.m;
    end
end