%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Main Control Script %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Clear the workspace and close all figures
clear;
close all;

% Define system parameters
numSensors = 3;            % Number of sensors in the system
numSolenoids = 4;          % Number of solenoids (actuators) in the system

% Create a struct to hold plotting options
plottingOptions = struct(...
    'enablePlotting', true, ...
    'enableSensorDataPlot', true, ...
    'enableSolenoidCurrentsPlot', true, ...
    'enableFilteredDataPlot', false, ...
    'enableDerivativeDataPlot', false, ...
    'maxDataPoints', 200);

% Create an instance of the TeensyComms class to communicate with the Teensy board
teensy = TeensyComms(numSensors, numSolenoids);

% Initialize the visualization (plots)
[plotHandles, fig, stopButton] = initializeVisualization(numSensors, numSolenoids, plottingOptions);

% Set the desired loop frequency and calculate the loop period
desiredFrequency = 200;             % Desired loop frequency in Hz
loopPeriod = 1 / desiredFrequency;  % Loop period in seconds

% Start the main control loop timer
loopStart = tic;

% Main control loop: runs while the figure window is open
while ishandle(fig)
    loopTime = toc(loopStart);
    
    if loopTime >= loopPeriod
        loopStart = tic;  % Reset the timer for the next loop iteration
        
        % Call the main loop function
        loop(teensy, plotHandles, plottingOptions);
    end
end

% After exiting the loop, retrieve buffered data from the Teensy
[timestamps, sensorData, inputData] = teensy.getBufferedData();

% Clean up: delete the Teensy object to close the serial port
delete(teensy);

disp('Script terminated.');

%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Main Control Loop %%%
%%%%%%%%%%%%%%%%%%%%%%%%%

function loop(teensy, plotHandles, plottingOptions)
    % Persistent variables to store data between iterations
    persistent data

    % Initialize persistent variables on the first call
    if isempty(data)
        data.solenoidCurrentsPrevEnd = zeros(1, 4);
        data.gamma = 1;  % Initial guess for gamma (feedthrough coefficient)
        data.SOLENOIDCURRENTSFILTERED = [];
        data.TIMESTAMPS = [];
        data.SENSORDATA = [];
    end

    %% Read Data from Teensy
    [timestamps, sensorData, solenoidCurrents] = teensy.readData();
    if isempty(timestamps)
        return;  % Skip this loop iteration if no data is received
    end

    %% Process Data
    currentTime = toc(teensy.startTime);

    % Phase 1: Initial phase (first 2 seconds)
    if currentTime < 2
        % Update the mean sensor output to remove bias
        teensy.updateMeans(sensorData);
        unbiasedData = sensorData;            % Data without bias correction
        filteredData = zeros(size(sensorData));
        dFilteredData = zeros(size(sensorData));

    % Phase 2: Calibration phase (2 to 3 seconds)
    elseif currentTime < 3
        % Apply a sawtooth signal to the solenoids to perturb the system
        u = (2^teensy.pwm_bit_size) * (sawtooth(40*(currentTime - 2)));
        controllerInput = [u, -u, u, -u];
        teensy.sendInputToTeensy(controllerInput);

        % Filter solenoid currents to match solenoid dynamics
        [solenoidCurrentsFiltered, data.solenoidCurrentsPrevEnd] = ...
            filterSolenoidCurrents(solenoidCurrents, data.solenoidCurrentsPrevEnd, timestamps);

        % Accumulate data for estimating gamma
        data.SOLENOIDCURRENTSFILTERED = [data.SOLENOIDCURRENTSFILTERED; solenoidCurrentsFiltered];
        data.TIMESTAMPS = [data.TIMESTAMPS; timestamps];
        data.SENSORDATA = [data.SENSORDATA; sensorData];

        % Estimate gamma (feedthrough coefficient) using optimization
        data.gamma = estimateGamma(teensy, data);

        % Compute unbiased data by removing feedthrough effects
        unbiasedData = computeUnbiasedData(sensorData, teensy, data.gamma, solenoidCurrentsFiltered, timestamps);
        filteredData = zeros(size(sensorData));
        dFilteredData = zeros(size(sensorData));

    % Phase 3: Control phase (after 3 seconds)
    else
        % Filter solenoid currents to compensate for dynamics
        [solenoidCurrentsFiltered, data.solenoidCurrentsPrevEnd] = ...
            filterSolenoidCurrents(solenoidCurrents, data.solenoidCurrentsPrevEnd, timestamps);

        % Remove feedthrough effects to get unbiased data
        unbiasedData = computeUnbiasedData(sensorData, teensy, data.gamma, solenoidCurrentsFiltered, timestamps);

        % Apply custom filter to the data
        [filteredData, dFilteredData] = applyCustomFilter(timestamps, unbiasedData);

        % Compute control input using the controller function
        controllerInput = controller(filteredData, dFilteredData, teensy.numSolenoids, timestamps);

        %% Send Control Input to Teensy
        % If the z-component of the sensor data exceeds a threshold, send control input
        if abs(mean(unbiasedData(:, 3))) > 3
            teensy.sendInputToTeensy(controllerInput);
        else
            teensy.sendInputToTeensy(zeros(size(controllerInput)));
        end
    end

    %% Update Visualization
    if plottingOptions.enablePlotting
        updateVisualization(timestamps, unbiasedData, solenoidCurrents, filteredData, dFilteredData, ...
                            plotHandles, plottingOptions);
    end

    %% Control the draw rate to a fixed frame rate (e.g., 30 Hz)
    persistent lastDrawTime
    if isempty(lastDrawTime)
        lastDrawTime = tic;  % Initialize the timer for draw updates
    end
    if toc(lastDrawTime) >= 1 / 30
        drawnow limitrate;   % Update the plots
        lastDrawTime = tic;  % Reset the draw timer
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Helper Functions   %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%

function [solenoidCurrentsFiltered, solenoidCurrentsPrevEnd] = ...
    filterSolenoidCurrents(solenoidCurrents, solenoidCurrentsPrevEnd, timestamps)
    % Filters the solenoid currents to match solenoid dynamics
    tau = 1.15e-3;  % Time constant for solenoid dynamics
    solenoidCurrentsFiltered = zeros(size(solenoidCurrents));
    solenoidCurrentsFiltered(1, :) = solenoidCurrentsPrevEnd;
    for k = 2:length(timestamps)
        Ts = timestamps(k) - timestamps(k - 1);
        alpha = Ts / (tau + Ts);
        solenoidCurrentsFiltered(k, :) = alpha * solenoidCurrents(k, :) + ...
                                         (1 - alpha) * solenoidCurrentsFiltered(k - 1, :);
    end
    solenoidCurrentsPrevEnd = solenoidCurrents(end, :);
end

function gamma = estimateGamma(teensy, data)
    % Estimates the feedthrough coefficient gamma using accumulated data
    objfun = @(gamma) sum(vecnorm(data.SENSORDATA ...
                    - teensy.getMeanOutput() ...
                    - gamma * [diff(data.SOLENOIDCURRENTSFILTERED(:, [2, 1]), 1, 2), ...
                               diff(data.SOLENOIDCURRENTSFILTERED(:, [4, 3]), 1, 2), ...
                               zeros(length(data.TIMESTAMPS), teensy.numSensors - 2)]));
    options = optimoptions('fminunc', 'Display', 'off');
    gamma = fminunc(objfun, data.gamma, options);
end

function unbiasedData = computeUnbiasedData(sensorData, teensy, gamma, solenoidCurrentsFiltered, timestamps)
    % Computes unbiased sensor data by removing feedthrough effects
    unbiasedData = sensorData - teensy.getMeanOutput() ...
                 - gamma * [diff(solenoidCurrentsFiltered(:, [2, 1]), 1, 2), ...
                            diff(solenoidCurrentsFiltered(:, [4, 3]), 1, 2), ...
                            zeros(length(timestamps), teensy.numSensors - 2)];
end

function [filteredData, dFilteredData] = applyCustomFilter(timestamps, unbiasedData)
    % Applies custom filtering to the data
    filterOrder = 5;
    alphaCoeffs = linspace(0.7, 0.01, filterOrder + 1);
    alphaCoeffs = alphaCoeffs / sum(alphaCoeffs); % Normalize to sum to 1
    dAlphaCoeffs = linspace(0.2, 0.05, filterOrder + 1);
    dAlphaCoeffs = dAlphaCoeffs / sum(dAlphaCoeffs); % Normalize to sum to 1

    [filteredData, dFilteredData] = filter(timestamps, unbiasedData, filterOrder, alphaCoeffs, dAlphaCoeffs);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Filter & Controller %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [filteredData, dFilteredData] = filter(timestamps, data, filterOrder, alphaCoeffs, dAlphaCoeffs)
    % Applies an nth-order IIR filter with median filtering to the data
    % Also computes the derivative of the filtered data

    persistent filteredDataPrev dFilteredDataPrev

    % Initialize persistent variables if empty
    if isempty(filteredDataPrev)
        filteredDataPrev = zeros(filterOrder, size(data, 2));  % Store previous filtered values
        dFilteredDataPrev = zeros(filterOrder, size(data, 2)); % Store previous derivative values
    end

    % Initialize output arrays
    filteredData = zeros(size(data));
    dFilteredData = zeros(size(data));
    filteredData(1:filterOrder, :) = filteredDataPrev;  % Use previous filtered values for initial points
    dFilteredData(1:filterOrder, :) = dFilteredDataPrev;

    % Apply median filter to data before IIR filtering
    for col = 1:size(data, 2)
        data(:, col) = medfilt1(data(:, col), 3);  % Median filter with window size of 3
    end

    % Apply nth-order IIR filter and compute derivative
    for k = (filterOrder + 1):size(data, 1)
        % IIR filter for data
        filteredData(k, :) = alphaCoeffs(1) * data(k, :);
        for j = 1:filterOrder
            filteredData(k, :) = filteredData(k, :) + alphaCoeffs(j + 1) * filteredData(k - j, :);
        end

        % Compute raw derivative
        rawDerivative = (filteredData(k, :) - filteredData(k - 1, :)) / (diff(timestamps(k - 1:k)) + 1e-5);

        % IIR filter for derivative
        dFilteredData(k, :) = dAlphaCoeffs(1) * rawDerivative;
        for j = 1:filterOrder
            dFilteredData(k, :) = dFilteredData(k, :) + dAlphaCoeffs(j + 1) * dFilteredData(k - j, :);
        end
    end

    % Update persistent variables for next iteration
    filteredDataPrev = filteredData(end - filterOrder + 1:end, :);
    dFilteredDataPrev = dFilteredData(end - filterOrder + 1:end, :);
end

function controllerInput = controller(filteredData, dFilteredData, numSolenoids, timestamps)
    % Proportional-Derivative (PD) controller to compute control inputs

    % Controller gains
    Kp = 750;  % Proportional gain
    Kd = 4;    % Derivative gain

    % Extract the x and y components of the filtered data and their derivatives
    x  = mean(filteredData(:, 1));
    dx = mean(dFilteredData(:, 1));

    y  = mean(filteredData(:, 2));
    dy = mean(dFilteredData(:, 2));

    % Compute control actions for x and y
    ux = -Kp * x - Kd * dx;
    uy = -Kp * y - Kd * dy;

    uxp =  ux;
    uxn = -ux;
    uyp =  uy;
    uyn = -uy;

    % Construct the controller input vector
    controllerInput = [uxp; uxn; uyp; uyn];

    % Ensure controller input is valid
    if any(abs(controllerInput) == inf) || any(isnan(controllerInput))
        controllerInput = zeros(size(controllerInput));
    end

    % Check that the controller output matches the system input size
    if length(controllerInput) ~= numSolenoids
        error('Controller output does not match size of system input.');
    end
end

%%%%%%%%%%%%%%%%%%%%%
%%% Visualization %%%
%%%%%%%%%%%%%%%%%%%%%

function updateVisualization(timestamps, sensorData, solenoidCurrents, filteredData, dFilteredData, ...
                             plotHandles, plottingOptions)
    % Updates the plots with the latest data

    if isempty(timestamps)
        return;  % Skip if no data
    end

    % Update sensor data plots
    if plottingOptions.enableSensorDataPlot
        for i = 1:length(plotHandles.sensorData)
            addpoints(plotHandles.sensorData(i), timestamps(end), sensorData(end, i));
        end
    end

    % Update solenoid current plots
    if plottingOptions.enableSolenoidCurrentsPlot
        for i = 1:length(plotHandles.solenoidCurrents)
            addpoints(plotHandles.solenoidCurrents(i), timestamps(end), solenoidCurrents(end, i));
        end
    end

    % Update filtered data plots
    if plottingOptions.enableFilteredDataPlot
        for i = 1:length(plotHandles.filteredData)
            addpoints(plotHandles.filteredData(i), timestamps(end), filteredData(end, i));
        end
    end

    % Update derivative data plots
    if plottingOptions.enableDerivativeDataPlot
        for i = 1:length(plotHandles.derivativeData)
            addpoints(plotHandles.derivativeData(i), timestamps(end), dFilteredData(end, i));
        end
    end

    % Refresh the plots with rate limiting
    drawnow limitrate;
end

function [plotHandles, fig, stopButton] = initializeVisualization(numSensors, numSolenoids, plottingOptions)
    % Initialize the visualization figures and plots

    % Initialize plot handles struct
    plotHandles = struct();

    if plottingOptions.enablePlotting
        % Create figure for visualization
        fig = figure('Name', 'Teensy Visualization', 'NumberTitle', 'off');
        set(fig, 'Units', 'normalized', 'OuterPosition', [0.05, 0.05, 0.8, 0.7]);

        % Create a panel for the stop button at the bottom of the figure
        buttonPanel = uipanel('Parent', fig, 'Units', 'normalized', 'Position', [0, 0, 1, 0.1]);

        % Create the stop button within the panel
        stopButton = uicontrol('Parent', buttonPanel, 'Style', 'pushbutton', 'String', 'Stop', ...
            'FontSize', 12, 'FontWeight', 'bold', 'Units', 'normalized', ...
            'Position', [0.45, 0.1, 0.1, 0.8], 'Callback', @(~, ~) close(fig));

        % Determine the number of subplots to create based on flags
        numPlots = plottingOptions.enableSensorDataPlot + plottingOptions.enableSolenoidCurrentsPlot + ...
                   plottingOptions.enableFilteredDataPlot + plottingOptions.enableDerivativeDataPlot;
        t = tiledlayout(fig, 1, numPlots, 'TileSpacing', 'compact', 'Padding', 'none');
        t.TileIndexing = 'rowmajor';
        t.Units = 'normalized';
        t.OuterPosition = [0.01, 0.2, 0.95, 0.7]; % Adjust position to be above the button panel

        % Create axes and animated lines based on flags
        plotIdx = 1;

        if plottingOptions.enableSensorDataPlot
            ax1 = nexttile(t, plotIdx);
            hold(ax1, 'on'); box(ax1, 'on');
            ylabel(ax1, 'Measurements [T]', 'Interpreter', 'latex', 'FontSize', 16);
            plotHandles.sensorData = gobjects(numSensors, 1);
            colors = lines(numSensors);
            for i = 1:numSensors
                plotHandles.sensorData(i) = animatedline(ax1, 'Color', colors(i, :), 'LineWidth', 2, ...
                    'MaximumNumPoints', plottingOptions.maxDataPoints);
            end
            plotIdx = plotIdx + 1;
        end

        if plottingOptions.enableSolenoidCurrentsPlot
            ax2 = nexttile(t, plotIdx);
            hold(ax2, 'on'); box(ax2, 'on');
            ylabel(ax2, 'Solenoid Currents [A]', 'Interpreter', 'latex', 'FontSize', 16);
            plotHandles.solenoidCurrents = gobjects(numSolenoids, 1);
            colors = lines(numSolenoids);
            for i = 1:numSolenoids
                plotHandles.solenoidCurrents(i) = animatedline(ax2, 'Color', colors(i, :), 'LineWidth', 2, ...
                    'MaximumNumPoints', plottingOptions.maxDataPoints);
            end
            plotIdx = plotIdx + 1;
        end

        if plottingOptions.enableFilteredDataPlot
            ax3 = nexttile(t, plotIdx);
            hold(ax3, 'on'); box(ax3, 'on');
            xlabel(ax3, 'Time [s]', 'Interpreter', 'latex', 'FontSize', 16);
            ylabel(ax3, 'Filtered Measurements [T]', 'Interpreter', 'latex', 'FontSize', 16);
            plotHandles.filteredData = gobjects(numSensors, 1);
            colors = lines(numSensors);
            for i = 1:numSensors
                plotHandles.filteredData(i) = animatedline(ax3, 'Color', colors(i, :), 'LineWidth', 2, ...
                    'MaximumNumPoints', plottingOptions.maxDataPoints);
            end
            plotIdx = plotIdx + 1;
        end

        if plottingOptions.enableDerivativeDataPlot
            ax4 = nexttile(t, plotIdx);
            hold(ax4, 'on'); box(ax4, 'on');
            xlabel(ax4, 'Time [s]', 'Interpreter', 'latex', 'FontSize', 16);
            ylabel(ax4, 'Derivative of Filtered Measurements [T/s]', 'Interpreter', 'latex', 'FontSize', 16);
            plotHandles.derivativeData = gobjects(numSensors, 1);
            colors = lines(numSensors);
            for i = 1:numSensors
                plotHandles.derivativeData(i) = animatedline(ax4, 'Color', colors(i, :), 'LineWidth', 2, ...
                    'MaximumNumPoints', plottingOptions.maxDataPoints);
            end
        end
    else
        % If plotting is disabled, create a minimal figure with a stop button
        fig = figure('Name', 'Teensy Visualization', 'NumberTitle', 'off', ...
            'Position', [100, 100, 400, 200]);

        stopButton = uicontrol('Style', 'pushbutton', 'String', 'Stop', ...
            'FontSize', 12, 'FontWeight', 'bold', 'Position', [20, 20, 360, 160], ...
            'Callback', @(~, ~) close(fig));
    end

    drawnow;
end

%%%%%%%%%%%%%%%%%%%%%%%
%%% End of Script   %%%
%%%%%%%%%%%%%%%%%%%%%%%
