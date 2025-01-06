%% TODO:
% - Ensure that it works with a controller
% - Add searching for port instead of using a fixed port (on the Matlab
% side, so that the user may choose port if there are several)
% -- This requires a separate matlab function
% - Ensure that the setting is set regardless of the port that is connected
% - Put everything into nice folders and make a minimal script for control
% - Ensure that this works on other computers

% - Save data and plot entire time series after a run
% - Make it so that the remapping of the input is happeing in Matlab
% (and maybe make that function tuned automatically)
% - Tune the direct feedthrough automatically
% - Make it so that they can choose sensor to get data from

% - OBS: The time vector will have problems after about 35 minutes!
% - OBS: Direct feedthrough de-biasing has to happen in the arduino file
% (to not have problems with delay)

function ModelValidation
    % Main function to read data from Teensy 4.1 and plot it
    plotWindowSize = 1; % in seconds
    maxDataPoints = 500; % Maximum number of data points to store
    withPlotting = false;
    
    % Initialize sharedData structure with default values
    persistent sharedData;
    
    sharedData.meanBx = 0;
    sharedData.meanBy = 0;
    sharedData.meanBz = 0;
    sharedData.ux = int16(0);
    sharedData.uy = int16(0);
    sharedData.startTime = tic;
    sharedData.previousTime = 0;
    sharedData.controlUpdateStartTime = tic;

    sharedData.index_ux = -255;
    sharedData.index_uy = -255;
    ux = 0;
    uy = 0;

    sharedData.BX = zeros(size(-255:255));
    sharedData.BY = zeros(size(-255:255));

    sharedData.num_samples = 0;

    % Initialize the plot and store handles in sharedData
    [sharedData.hX_old, sharedData.hY_old, sharedData.hZ_old, sharedData.hX_new, sharedData.hY_new, sharedData.hZ_new, sharedData.fig] = initializePlot();

    % Initialize serial port
    % Open the COM port
    com_port_operations_mex('open', 'COM9');

    % Set COM port timeouts
    com_port_operations_mex('set_timeout');

    figure(1);

    % Set up a timer for polling the serial port
    t = timer('ExecutionMode', 'fixedRate', 'Period', 1/200, 'TimerFcn', @(~, ~) pollSerialPort());
    start(t);

    % Wait for the figure window to close
    waitfor(sharedData.fig);
    
    % Cleanup code, if necessary
    stop(t);
    delete(t);

    % Close the COM port
    com_port_operations_mex('close');

    % Save data
    assignin('base', 'sharedData', sharedData);

    % Terminate script
    disp('Script terminated.');

    %% Timer function to poll the serial port
    function pollSerialPort()
        % Receive data
        try
            rawData = com_port_operations_mex('read');  % Read 12 bytes (3 floats)

            % Convert raw data to bytes and then to floats
            rawData = uint8(rawData);  % Ensure it's treated as raw bytes
            data = double(reshape(typecast(rawData, 'single'),4,[])');  % Convert to an array of 3 floats
            x = data(:,1:3);
            timeStamps = data(:,4)/1e6;
        catch
            warning("Connection lost. Searching for available ports again...");
            return; % Exit the function to allow reconnection in the main loop
        end

        % Process data
        if ~isempty(x)
            %% Sample frequency calculations
            elapsedTime = toc(sharedData.startTime); % Measure elapsed time using toc
            newTimestamps = timeStamps';
            sharedData.previousTime = elapsedTime;

            %% CONTROL UPDATE
            if toc(sharedData.controlUpdateStartTime) > 0.04
                fprintf('%.f | %.f\n', sharedData.index_ux, sharedData.index_uy)
                if sharedData.index_ux <= 255
                    ux = sharedData.index_ux;                              
                    sharedData.index_ux = sharedData.index_ux + 5;
                    sharedData.num_samples = 0;
                elseif sharedData.index_ux > 255 && sharedData.index_uy <= 255
                    ux = 0;
                    uy = sharedData.index_uy;
                    sharedData.index_uy = sharedData.index_uy + 5;
                    sharedData.num_samples = 0;
                else
                    % Stop the timer when both indices are done
                    data = typecast(int16([0, 0]), 'uint8');
                    com_port_operations_mex('write', data);
                    close(sharedData.fig)
                    return;
                end

                % Change input to next step
                sharedData.ux = int16(min(max(ux, -255), 255)); 
                sharedData.uy = int16(min(max(uy, -255), 255)); 

                data = typecast(int16([sharedData.ux, sharedData.uy]), 'uint8');
                com_port_operations_mex('write', data);

                % Update timer
                sharedData.controlUpdateStartTime = tic;
            else
                % Save average of data
                sharedData.num_samples = sharedData.num_samples + 1;
                
                if sharedData.index_ux <=255
                   sharedData.BX(sharedData.index_ux+256) = sharedData.BX(sharedData.index_ux+256) + (mean(x(:,1)) - sharedData.BX(sharedData.index_ux+256))/sharedData.num_samples; 
                elseif sharedData.index_uy <=255
                   sharedData.BY(sharedData.index_uy+256) = sharedData.BY(sharedData.index_uy+256) + (mean(x(:,2)) - sharedData.BY(sharedData.index_uy+256))/sharedData.num_samples; 
                end
                

            end

            %% Plot the data
            if withPlotting
                % Check if the figure is still open before trying to update the plot
                if isvalid(sharedData.fig)
                    updatePlot(sharedData.hX_old, sharedData.hY_old, sharedData.hZ_old, sharedData.hX_new, sharedData.hY_new, sharedData.hZ_new, newTimestamps, x);
                    xlim([max(0, newTimestamps(end) - plotWindowSize), newTimestamps(end)]);
            
                    % Update the plot less frequently to reduce overhead
                    drawnow limitrate;
                else
                    % Disable the callback to prevent further errors
                    stop(t);
                    disp('Figure closed, stopping data processing.');
                    return;
                end
            end
        end
    end

    %% Initialize Plot
    function [hX_old, hY_old, hZ_old, hX_new, hY_new, hZ_new, fig] = initializePlot()
        % Initialize animated lines for plotting
        fig = figure(1);
        set(gcf, 'Position', [500, 500, 600, 300]);
        clf; 
        hold on; box on;
        hX_old = animatedline('Color', 'r', 'linewidth', 2, 'MaximumNumPoints', maxDataPoints);
        hY_old = animatedline('Color', 'b', 'linewidth', 2, 'MaximumNumPoints', maxDataPoints);
        hZ_old = animatedline('Color', 'g', 'linewidth', 2, 'MaximumNumPoints', maxDataPoints);
        hX_new = animatedline('Color', 'k', 'linewidth', 2, 'MaximumNumPoints', maxDataPoints);
        hY_new = animatedline('Color', 'k', 'linewidth', 2, 'MaximumNumPoints', maxDataPoints);
        hZ_new = animatedline('Color', 'k', 'linewidth', 2, 'MaximumNumPoints', maxDataPoints);
        hold off;

        drawnow
    end

    %% Update Plot
    function updatePlot(hX_old, hY_old, hZ_old, hX_new, hY_new, hZ_new, newTimestamps, x)
        % Function to update the plot with new data
        
        if ishandle(hX_old) && ishandle(hY_old) && ishandle(hZ_old) ...
           && ishandle(hX_new) && ishandle(hY_new) && ishandle(hZ_new)
            
            % Append new points to the old animated lines
            addpoints(hX_old, newTimestamps, x(:, 1));
            addpoints(hY_old, newTimestamps, x(:, 2));
            addpoints(hZ_old, newTimestamps, x(:, 3));
    
            % Append new points to the new animated lines
            clearpoints(hX_new);
            clearpoints(hY_new);
            clearpoints(hZ_new);
            addpoints(hX_new, newTimestamps, x(:, 1));
            addpoints(hY_new, newTimestamps, x(:, 2));
            addpoints(hZ_new, newTimestamps, x(:, 3));
        else
            warning('One or more plot handles are invalid.');
        end
    end
end