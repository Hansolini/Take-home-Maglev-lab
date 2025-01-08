function c = computeSolenoidRadiusCorrectionFactor(params,modelName)
    % Find equilibrium (used in objective function)
    zEq = computeSystemEquilibria(params,'fast');

    try
        zEq = zEq(1);
    catch
        disp('No real equilibrium for this system')
        return;
    end

    % Optimize
    options = optimoptions('fmincon', 'Display', 'off');
    c = fmincon(@(c) objfun(c,zEq,params,modelName),1,[],[],[],[],0,inf,[],options);

    % Helping functions
    function fz = computeVerticalForce(z,params,modelName)
        u = 100*ones(length(params.solenoids.r),1); % Non-zero u in order to test solenoids
        
        % Compute normal graf
        fz = zeros(size(z));
        for j = 1:length(z)
            x = [0;0;z(j);zeros(9,1)];
            [~,~,fz] = computeForceAndTorque(x,u,params,modelName);
        end
        fz = fz - 9.81*params.magnet.m;
    end

    function J = objfun(c,zEq,params,modelName)
        paramsCorrected = params;
        paramsCorrected.solenoids.r = c*paramsCorrected.solenoids.r;

        J = norm(computeVerticalForce(zEq,params,'filament') - computeVerticalForce(zEq,paramsCorrected,modelName));
    end

end