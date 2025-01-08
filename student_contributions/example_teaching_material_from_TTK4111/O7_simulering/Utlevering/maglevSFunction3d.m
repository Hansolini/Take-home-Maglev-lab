function maglevSFunction3d(block)
    setup(block);
end

function setup(block)
    % Register the number of input, output, and continuous states
    block.NumInputPorts  = 1;  % Control input u
    block.NumOutputPorts = 2;  % State x and measurement h(x,u)
    block.NumContStates  = 10;  % Number of continuous states (assuming x has 6 elements)

    % Setup port properties
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    % Configure input port
    block.InputPort(1).Dimensions        = 4;       % Control input dimension
    block.InputPort(1).DatatypeID        = 0;       % double
    block.InputPort(1).SamplingMode      = 'Sample';
    block.InputPort(1).DirectFeedthrough = false;   % Outputs do not depend on inputs

    % Configure output ports
    block.OutputPort(1).Dimensions   = 10;          % State output dimension
    block.OutputPort(1).DatatypeID   = 0;           % double
    block.OutputPort(1).SamplingMode = 'Sample';

    block.OutputPort(2).Dimensions   = 3;           % Measurement output dimension
    block.OutputPort(2).DatatypeID   = 0;           % double
    block.OutputPort(2).SamplingMode = 'Sample';

    % Set sample time
    block.SampleTimes = [0 0]; % Continuous sample time

    % Register parameters
    block.NumDialogPrms = 0; % params structure

    % Set SimState compliance
    block.SimStateCompliance = 'DefaultSimState';

    % Register methods
    block.RegBlockMethod('InitializeConditions', @InitializeConditions);
    block.RegBlockMethod('Outputs',              @Outputs);
    block.RegBlockMethod('Derivatives',          @Derivatives);
    block.RegBlockMethod('Terminate',            @Terminate); % Optional: To close figures
end

function InitializeConditions(block)
    % Retrieve parameters
    params = getParams();

    % Set initial conditions
    if isfield(params, 'initial_conditions')
        x0 = params.initial_conditions;
    else
        % Default initial conditions
        z0 = 0.05;
        x0 = [0.005; 0; z0; zeros(7,1)]; % Adjusted to match dimensions
    end

    % Initialize continuous states
    block.ContStates.Data = x0;

    % Initialize plotting
    init_plot(x0);
end

function Outputs(block)    
    % Retrieve the current state (x)
    x = block.ContStates.Data;
    u = block.InputPort(1).Data;

    % Retrieve parameters
    params = getParams();

    % Output the current state
    block.OutputPort(1).Data = x; % Current state x

    % Calculate and output the measurement
    measurements = maglevSystemMeasurements3d(x, u, params);
    block.OutputPort(2).Data = measurements; % h(x, u)

    % Update the plot
    update_plot(x);

    % Check for deviation and stop simulation if necessary
    check_deviation_and_stop_simulation(x);
end

function Derivatives(block)
    % Retrieve the current state (x) and control input (u)
    x = block.ContStates.Data;
    u = block.InputPort(1).Data;

    % Retrieve parameters
    params = getParams();

    % Calculate the state derivatives (f(x,u))
    dxdt = maglevSystemDynamics3d(x, u, params);

    % Set the calculated derivatives
    block.Derivatives.Data = dxdt;
end

function Terminate(~)
    % Optional: Close figures if needed
    MaglevPlotFig = getappdata(0, 'MaglevPlotFig');
    if ishandle(MaglevPlotFig)
        close(MaglevPlotFig);
    end
    % Clean up application data
    rmappdata(0, 'MaglevPlotFig');
    rmappdata(0, 'MaglevHandle');
    rmappdata(0, 'LastUpdateTime');
end

%% Plotting Functions
function init_plot(x0)
    % Persistent variables for plotting
    MaglevPlotFig = getappdata(0, 'MaglevPlotFig');

    if isempty(MaglevPlotFig) || ~ishandle(MaglevPlotFig)
        % Load parameters
        params = getParams();

        % Create figure and axes
        MaglevPlotFig = figure('Name', 'Maglev Animation', 'NumberTitle', 'off');
        MaglevPlotAxes = axes('Parent', MaglevPlotFig);
        grid(MaglevPlotAxes, 'on'); hold(MaglevPlotAxes, 'on'); box(MaglevPlotAxes, 'on');
        daspect(MaglevPlotAxes, [1, 1, 1]);
        xlim(MaglevPlotAxes, [-0.1, 0.1]);
        ylim(MaglevPlotAxes, [-0.1, 0.1]);
        zlim(MaglevPlotAxes, [-0.05, 0.1]);
        xlabel(MaglevPlotAxes, '$x$','Interpreter','latex','FontSize',14);
        ylabel(MaglevPlotAxes, '$y$','Interpreter','latex','FontSize',14);
        zlabel(MaglevPlotAxes, '$z$','Interpreter','latex','FontSize',14);
        view([45,15]);

        % Draw base
        plotBaseArtistic(params);

        % Draw maglev
        MaglevHandle = plotMagnetArtistic(0,0,0,0,0,0,params);
        updatePositionOfObject(MaglevHandle,x0(1),x0(2),x0(3),x0(4),x0(5),0);
        
        % Store handles in application data
        setappdata(0, 'MaglevPlotFig', MaglevPlotFig);
        setappdata(0, 'MaglevHandle', MaglevHandle);
        setappdata(0, 'LastUpdateTime', 0);
    end
end

function update_plot(x)
    % Retrieve persistent data
    MaglevHandle = getappdata(0, 'MaglevHandle');

    % Update maglev position and orientation
    updatePositionOfObject(MaglevHandle,x(1),x(2),x(3),x(4),x(5),0);
    drawnow limitrate nocallbacks; % Efficient drawing
end

%% Deviation Check Function
function check_deviation_and_stop_simulation(x)
    deviation_threshold_x = 0.05;
    deviation_threshold_y = 0.05;
    deviation_threshold_z = 0.05;
    deviation_threshold_theta = pi/2;
    
    % Calculate deviation
    deviation_x = norm(x(1));
    deviation_y = norm(x(2));
    deviation_z = norm(x(3) - 0.03);
    deviation_theta_x = norm(x(4));
    deviation_theta_y = norm(x(5));

    % Check if deviation exceeds threshold
    if deviation_x > deviation_threshold_x
        % Stop the simulation
        warning('Simulation stopped: State x deviated too much from the desired value.');
        set_param(bdroot, 'SimulationCommand', 'stop');
    end

    if deviation_y > deviation_threshold_y
        % Stop the simulation
        warning('Simulation stopped: State y deviated too much from the desired value.');
        set_param(bdroot, 'SimulationCommand', 'stop');
    end

    if deviation_z > deviation_threshold_z
        % Stop the simulation
        warning('Simulation stopped: State z deviated too much from the desired value.');
        set_param(bdroot, 'SimulationCommand', 'stop');
    end

    if deviation_theta_x > deviation_threshold_theta
        % Stop the simulation
        warning('Simulation stopped: State theta x deviated too much from the desired value.');
        set_param(bdroot, 'SimulationCommand', 'stop');
    end

    if deviation_theta_y > deviation_threshold_theta
        % Stop the simulation
        warning('Simulation stopped: State theta y deviated too much from the desired value.');
        set_param(bdroot, 'SimulationCommand', 'stop');
    end
end

function data = getParams()
    persistent params

    if isempty(params)
        parameters;
    end
    
    data = params;
end


function dx = maglevSystemDynamics3d(x,u,params)
    % ### HACK ###
    x_full = [
        x(1),x(2),x(3),x(4),x(5),0,...
        x(6),x(7),x(8),x(9),x(10),0
        ]';
    u_full = [u(1), u(2), u(3), u(4)]';
    % ##########################
    
    dx = maglevSystemDynamics(x_full,u_full,params);
    
    % ### HACK ###
    dx = dx([1,2,3,4,5,7,8,9,10,11]');
    % ##########################
end

function y = maglevSystemMeasurements3d(x,u,params)
    % ### HACK ###
    x_full = [
        x(1),x(2),x(3),x(4),x(5),0,...
        x(6),x(7),x(8),x(9),x(10),0
        ]';
    u_full = [u(1), u(2), u(3), u(4)]';
    % ##########################
    
    y = maglevSystemMeasurements(x_full,u_full,params);
    
    % ### HACK from 3d to 2d ###
    y = y([1,2,3]');
    % ##########################
end
