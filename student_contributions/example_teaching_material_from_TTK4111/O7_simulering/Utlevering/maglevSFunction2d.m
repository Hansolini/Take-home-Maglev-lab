function maglevSFunction2d(block)
    setup(block);
end

function setup(block)
    % Register the number of input, output, and continuous states
    block.NumInputPorts  = 1;  % Control input u
    block.NumOutputPorts = 2;  % State x and measurement h(x,u)
    block.NumContStates  = 6;  % Number of continuous states (assuming x has 6 elements)

    % Setup port properties
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    % Configure input port
    block.InputPort(1).Dimensions        = 2;       % Control input dimension
    block.InputPort(1).DatatypeID        = 0;       % double
    block.InputPort(1).SamplingMode      = 'Sample';
    block.InputPort(1).DirectFeedthrough = false;   % Outputs do not depend on inputs

    % Configure output ports
    block.OutputPort(1).Dimensions   = 6;           % State output dimension
    block.OutputPort(1).DatatypeID   = 0;           % double
    block.OutputPort(1).SamplingMode = 'Sample';

    block.OutputPort(2).Dimensions   = 2;           % Measurement output dimension
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
        x0 = [0.01; z0; 0; zeros(3,1)]; % Adjusted to match dimensions
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
    measurements = maglevSystemMeasurements2d(x, u, params);
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
    dxdt = maglevSystemDynamics2d(x, u, params);

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
        % Create figure and axes
        MaglevPlotFig = figure('Name', 'Maglev Animation', 'NumberTitle', 'off');
        MaglevPlotAxes = axes('Parent', MaglevPlotFig);
        grid(MaglevPlotAxes, 'on'); hold(MaglevPlotAxes, 'on'); box(MaglevPlotAxes, 'on');
        daspect(MaglevPlotAxes, [1, 1, 1]);
        xlim(MaglevPlotAxes, [-0.1, 0.1]);
        ylim(MaglevPlotAxes, [-0.05, 0.1]);
        xlabel(MaglevPlotAxes, '$x$','Interpreter','latex','FontSize',14);
        ylabel(MaglevPlotAxes, '$z$','Interpreter','latex','FontSize',14);

        % Draw ground
        yline(MaglevPlotAxes, 0, 'k', 'LineWidth', 2);

        % Draw solenoids
        w = 2*0.0092;
        h = 0.0120;
        rectangle('Parent', MaglevPlotAxes, 'Position',[0.02-w/2,0,w,h], 'EdgeColor', 'k', 'FaceColor',[0.72, 0.45, 0.2], 'LineWidth',2);
        rectangle('Parent', MaglevPlotAxes, 'Position',[-0.02-w/2,0,w,h], 'EdgeColor', 'k', 'FaceColor',[0.72, 0.45, 0.2], 'LineWidth',2);

        % Draw maglev vehicle
        MaglevHandle = create_maglev(MaglevPlotAxes);
        set(MaglevHandle, 'Matrix', makehgtform('translate', [x0(1), x0(2), 0], 'zrotate', -x0(3)));

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
    set(MaglevHandle, 'Matrix', makehgtform('translate', [x(1), x(2), 0], 'zrotate', x(3)));
    drawnow limitrate nocallbacks; % Efficient drawing
end

function MaglevHandle = create_maglev(ax)
    % Derived parameters
    w_body = 2*0.0250;
    h_body = 0.0050;
    r_center = 0.001;
    linewidth = 1;

    % Maglev transform object
    MaglevHandle = hgtransform('Parent', ax);

    % Body
    rectangle('Parent', MaglevHandle, 'Position', [-w_body/2, -h_body/2, w_body, h_body], ...
        'EdgeColor', 'k', 'FaceColor', [0.5, 0.5, 0.5], 'LineWidth', linewidth);

    % Center marker
    rectangle('Parent', MaglevHandle, 'Position', [-r_center/2, -r_center/2, r_center, r_center], ...
        'Curvature', [1, 1], 'EdgeColor', 'k', 'FaceColor', 'k', 'LineWidth', linewidth);
end

%% Deviation Check Function
function check_deviation_and_stop_simulation(x)
    deviation_threshold_x = 0.05;
    deviation_threshold_z = 0.05;
    deviation_threshold_theta = pi/2;
    
    % Calculate deviation
    deviation_x = norm(x(1));
    deviation_z = norm(x(2) - 0.03);
    deviation_theta = norm(x(3));

    % Check if deviation exceeds threshold
    if deviation_x > deviation_threshold_x
        % Stop the simulation
        warning('Simulation stopped: State x deviated too much from the desired value.');
        set_param(bdroot, 'SimulationCommand', 'stop');
    end

    if deviation_z > deviation_threshold_z
        % Stop the simulation
        warning('Simulation stopped: State z deviated too much from the desired value.');
        set_param(bdroot, 'SimulationCommand', 'stop');
    end

    if deviation_theta > deviation_threshold_theta
        % Stop the simulation
        warning('Simulation stopped: State theta deviated too much from the desired value.');
        set_param(bdroot, 'SimulationCommand', 'stop');
    end
end

%% Load parameters function
function data = getParams()
    persistent params

    if isempty(params)
        parameters;
    end
    
    data = params;
end

%% System dynamics for 2D system
function dx = maglevSystemDynamics2d(x,u,params)
    % ### HACK from 2d to 3d ###
    x_full = [
        x(1),0,x(2),0,x(3),0,...
        x(4),0,x(5),0,x(6),0
        ]';
    u_full = [u(1), u(2), 0, 0]';
    % ##########################
    
    dx = maglevSystemDynamics(x_full,u_full,params);
    
    % ### HACK from 3d to 2d ###
    dx = dx([1,3,5,7,9,11]');
    % ##########################
end

function y = maglevSystemMeasurements2d(x,u,params)
    % ### HACK from 2d to 3d ###
    x_full = [
        x(1),0,x(2),0,x(3),0,...
        x(4),0,x(5),0,x(6),0
        ]';
    u_full = [u(1), u(2), 0, 0]';
    % ##########################
    
    y = maglevSystemMeasurements(x_full,u_full,params);
    
    % ### HACK from 3d to 2d ###
    y = y([1,3]');
    % ##########################
end