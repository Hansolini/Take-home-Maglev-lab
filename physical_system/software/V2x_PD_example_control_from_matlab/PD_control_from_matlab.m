%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                           PD Control with Matlab                    %%%
%%%                             for Maggy V2.6                          %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% DESCRIPTION:
% This script implements a PD controller in MATLAB to interact with Maggy 
% V2.6. It reads sensor data, processes it, applies filtering, computes 
% control inputs, and sends commands back to the Teensy at a high frequency.

% KEY FEATURES:
% - Real-time data acquisition and control at 200 Hz
% - Data filtering and bias removal
% - PD control computation and feedback
% - Optional real-time data visualization

% USAGE:
% - Set the number of sensors, solenoids, and plotting options.
% - Run the script to start the control loop; use the stop button to terminate.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% TODO:
% - Chekc if it is possible to induce rotation in simulation
% - There is a bug where sometimes the filtered data becomes nan/inf
 
clc; clear all; close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Main Control Script %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define number of sensors, solenoids, and plotting options
numSensors = 3;
numSolenoids = 4;
maxDataPoints = 500;
enablePlotting = true;

% Create an instance of the TeensyComms class with specified configuration
teensy = TeensyComms(numSensors, numSolenoids);

% Initialize the plot for visualization
[plotHandles, fig, stopButton] = initializePlot(maxDataPoints, enablePlotting, numSensors);

% Set desired loop frequency and calculate period
desiredFrequency = 200;  % Desired loop frequency in Hz
loopPeriod = 1 / desiredFrequency;  % Loop period in seconds

tic;

% Main control loop using tic and toc
loopStart = tic;
while ishandle(fig)  % Run loop while the figure is open
    loopTime = toc(loopStart);

    if loopTime >= loopPeriod
        fprintf('%.f\n', 1/toc);
        tic

        loopStart = tic;  % Reset the timer for the next loop
        loop(teensy, plotHandles, enablePlotting);  % Call the control loop function
    end
end

% Saving data from run
[timestamps, sensorData,inputData] = teensy.getBufferedData();

% Cleanup code
delete(teensy);  % Calls the destructor to close the serial port

disp('Script terminated.');

%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Main Control Loop %%%
%%%%%%%%%%%%%%%%%%%%%%%%%
function loop(teensy, plotHandles, enablePlotting)
    %% Read Data
    % Receives data from Teensy
    [timestamps, sensorData, solenoidCurrents] = teensy.readData();

    if isempty(timestamps)
        return;  % Skip loop iteration if no data
    end

    %% Process data
    if toc(teensy.startTime) < 2 % Computing mean measurements before starting control
        teensy.updateMeans(sensorData);
        unbiasedData = sensorData;
    else % Actual control loop
        % Remove bias in data
        unbiasedData = sensorData - teensy.getMeanOutput() - 1.43/255*[solenoidCurrents(:,[1,3]), zeros(length(timestamps), teensy.numSensors - 2)];
        
        % Filter data
        [filteredData, dFilteredData] = filter(timestamps, unbiasedData);

        % Compute control input
        controllerInput = controller(filteredData, dFilteredData, teensy.numSolenoids);
    
        %% Send Input Data to Teensy
        if abs(mean(unbiasedData(:,3))) > 3
            teensy.sendInputToTeensy(controllerInput);
        end
    end

    %% Update Visualization    
    % Updates plots or figures with current data, only if plotting is enabled
    if enablePlotting
        updateVisualization(timestamps, unbiasedData, plotHandles);
    end

    % Control the draw rate to a fixed frame rate (e.g., 30 Hz)
    persistent lastDrawTime
    if isempty(lastDrawTime)
        lastDrawTime = tic;  % Initialize the timer for draw updates
    end
    if toc(lastDrawTime) >= 1/30
        drawnow limitrate;
        lastDrawTime = tic;  % Reset the draw timer
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Controller & Filter %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [filteredData, dFilteredData] = filter(timestamps, data)
    % IIR Filter + numerical differentiation
    ALPHA = 0.1;  % Low-pass filter constant
    DALPHA = 0.02;  % Derivative filter constant

    persistent filteredDataPrevEnd dFilteredDataPrevEnd

    % Initialize persistent variables if they are empty
    if isempty(filteredDataPrevEnd)
        filteredDataPrevEnd = zeros(1, size(data, 2));
        dFilteredDataPrevEnd = zeros(1, size(data, 2));
    end

    % Initialize output data arrays
    filteredData = zeros(size(data));
    dFilteredData = zeros(size(data));
    filteredData(1, :) = filteredDataPrevEnd;
    dFilteredData(1, :) = dFilteredDataPrevEnd;

    % Filtering and differentiation
    for k = 2:size(data, 1)
        filteredData(k, :) = ALPHA * data(k, :) + (1 - ALPHA) * filteredData(k - 1, :);
        dFilteredData(k, :) = DALPHA * (filteredData(k, :) - filteredData(k - 1, :)) / diff(timestamps(k - 1:k) + 1e-4) + ...
                             (1 - DALPHA) * dFilteredData(k - 1, :);
    end

    % Update persistent variables for continuity in subsequent calls
    filteredDataPrevEnd = filteredData(end, :);
    dFilteredDataPrevEnd = dFilteredData(end, :);
end

function controllerInput = controller(filteredData, dFilteredData, sizeInput)
    % PD controller
    Kp = 222;  % Proportional gain
    Kd = 2;    % Derivative gain

    % Assume filteredData and dFilteredData are matrices with multiple sensor readings
    % Use median to reduce noise effect on control inputs
    x  = median(filteredData(:, 1));
    dx = median(dFilteredData(:, 1));

    y  = median(filteredData(:, 2));
    dy = median(dFilteredData(:, 2));

    ux = -Kp * x - Kd * dx;
    uy = -Kp * y - Kd * dy;

    % Compute PD control input
    controllerInput = [ux, -ux, uy, -uy]';

    if any(abs(controllerInput) == inf)
        controllerInput = zeros(size(controllerInput));
    end

    if any(isnan(controllerInput))
        controllerInput = zeros(size(controllerInput));
    end


    if length(controllerInput) ~= sizeInput
        error('Controller output does not match size of system input.')
    end
end

%%%%%%%%%%%%%%%%%%%%%
%%% Visualization %%%
%%%%%%%%%%%%%%%%%%%%%
function updateVisualization(timestamps, sensorData, plotHandles)
    % Update visualization with provided timestamps and sensor data
    % This function updates plots with the provided data
    
    if isempty(timestamps)
        return;  % Skip if no data
    end

    % Update all sensor data plots dynamically based on the number of sensors
    for i = 1:length(plotHandles)
        addpoints(plotHandles(i), timestamps(end), sensorData(end, i)); % Add data for each sensor
    end

    xlim([min(timestamps) - 2, max(timestamps)])
end

function [plotHandles, fig, stopButton] = initializePlot(maxDataPoints, enablePlotting, numSensors)
    % Initialize animated lines for plotting and return them in a single array
    % Includes a larger stop button with bold text to close the figure and stop the script

    if enablePlotting
        fig = figure('Name', 'Teensy Visualization', 'NumberTitle', 'off', ...
            'Position', [100, 100, 800, 600]);  % Larger size to include plots and button
        
        % Create an axes for plotting
        ax = axes('Parent', fig, 'Position', [0.1, 0.3, 0.8, 0.6]);  % Adjust position to leave space for the button
        hold(ax, 'on'); box(ax, 'on');
        xlabel('Time [s]', 'interpreter', 'latex','fontsize',16)
        ylabel('Measurements [T]', 'interpreter', 'latex','fontsize',16)

        % Create the stop button above the axes
        stopButton = uicontrol('Style', 'pushbutton', 'String', 'Stop', ...
            'FontSize', 12, 'FontWeight', 'bold', 'Position', [300, 550, 200, 40], ...  % Larger size and bold text
            'Callback', @(~, ~) close(fig));
        
        % Dynamically create plot handles for all sensors
        plotHandles = gobjects(numSensors, 1);  % Preallocate array for plot handles based on number of sensors
        colors = lines(numSensors);  % Generate a set of distinct colors for the plots

        for i = 1:numSensors
            plotHandles(i) = animatedline(ax, 'Color', colors(i, :), 'linewidth', 2, ...
                'MaximumNumPoints', maxDataPoints);  % Create animated line for each sensor
        end
    else
        fig = figure('Name', 'Teensy Visualization', 'NumberTitle', 'off', ...
            'Position', [100, 100, 400, 200]);  % Adjust figure size for button only
        
        % Create a stop button that fills the figure
        stopButton = uicontrol('Style', 'pushbutton', 'String', 'Stop', ...
            'FontSize', 12, 'FontWeight', 'bold', ...
            'Position', [20, 20, 360, 160], ...  % Adjust button to fill the figure
            'Callback', @(~, ~) close(fig));
        
        plotHandles = [];  % No plot handles needed when plotting is disabled
    end

    drawnow;
end

