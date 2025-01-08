function y = maglevSystemMeasurements(x, u, params)
    % MAGLEVSYSTEMMEASUREMENTS implements the function h in the ODE
    %   dxdt = f(x,u); 
    %      y = h(x,u);
    % defining the sensor measurements of a magnetic levitation system.
    %
    % Example:
    %   params; % Load parameters from a file or set manually
    %   x = [0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0]';
    %   u = [1, 0, -1, 0]';
    %
    %   h = @(x, u) maglevSystemMeasurements(x, u, params);
    %   y = h(x, u);
    %
    % See also MAGLEVSYSTEMDYNAMICS, COMPUTEFIELDTOTAL.

    % Defining position of sensor
    pSensor = [0;0;0];

    % Translating sensor to put magnet in the origin
    pTranslated = pSensor - x(1:3);

    % Rotating sensor position into local magnet frame
    R = rot(x(4), x(5), x(6));
    pRotated = R'*pTranslated;

    % Compute the current in the loop
    I = params.magnet.J/params.physical.mu0*params.magnet.l;

    % Compute magnetic field in the local frame
    [bxMagnet, byMagnet, bzMagnet] = computeFieldCircularWireCartesian( ...
        pRotated(1,:), ...
        pRotated(2,:), ...
        pRotated(3,:), ...
        params.magnet.r, ...
        I, ...
        params.physical.mu0);

    % Rotate magnetic field vectors to global frame (no translation needed)
    bMagnet = R*[bxMagnet; byMagnet; bzMagnet];

    % Separate field components
    bxMagnet = bMagnet(1, :);
    byMagnet = bMagnet(2, :);
    bzMagnet = bMagnet(3, :);

    % Field from base (set to zero if not needed)
    bxBase = 0;
    byBase = 0;
    bzBase = 0;

    % Uncomment if computeFieldBase is required
    % [bxBase, byBase, bzBase] = computeFieldBase(x, y, z, u, params);

    % Compute total field
    bx = bxBase + bxMagnet;
    by = byBase + byMagnet;
    bz = bzBase + bzMagnet;

    % Format output as a single column vector
    y = reshape([bx(:)'; by(:)'; bz(:)'], 3*numel(bx), 1);
end
