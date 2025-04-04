%% List available COM ports and let the user choose one
availablePorts = serialportlist("available");
if isempty(availablePorts)
    error("No available serial ports detected.");
end

disp("Available COM ports:");
for i = 1:length(availablePorts)
    fprintf('%d: %s\n', i, availablePorts(i));
end

portIndex = input("Enter the number of the COM port to use: ");
if isempty(portIndex) || portIndex < 1 || portIndex > length(availablePorts)
    error("Invalid COM port selection.");
end
comPort = availablePorts(portIndex);
fprintf("Using COM port: %s\n", comPort);

%% Specify the number of sensors expected
numSensors = input("Enter the number of sensors expected: ");
if isempty(numSensors) || numSensors < 1
    error("Invalid number of sensors specified.");
end

% Calculate expected number of fields: 
% 5 fixed fields (time, Ix_plus, Ix_minus, Iy_plus, Iy_minus) + 3 fields per sensor.
expectedFields = 5 + 3*numSensors;

%% Serial Port Setup and Data Logging Parameters
% Although baud rate is less critical for USB communication, it is still required
baudRate = 9600;       
readDuration = 10;     % Desired data duration (in seconds, relative time)

% Create and configure the serial port object
s = serialport(comPort, baudRate);
configureTerminator(s, "CR/LF");
pause(2);  % Pause to allow the Arduino to reset
flush(s);

%% Initialize vectors for data logging
t = [];        % Relative time in seconds
Ix_plus = [];  % Current Ix_plus
Ix_minus = []; % Current Ix_minus
Iy_plus = [];  % Current Iy_plus
Iy_minus = []; % Current Iy_minus

% Preallocate a cell array to store sensor magnetic field data
% Each cell will have fields: bx, by, bz.
sensorData = cell(numSensors,1);
for sensor = 1:numSensors
    sensorData{sensor}.bx = [];
    sensorData{sensor}.by = [];
    sensorData{sensor}.bz = [];
end

startTimeSet = false;  % Flag to record the first sample time
disp("Starting data collection based on sensor time...");

%% Data Collection Loop Based on Relative Sensor Time
while true
    if s.NumBytesAvailable > 0
        dataLine = strtrim(readline(s));
        disp(dataLine);
        
        % Split the line by commas
        fields = split(dataLine, ',');
        % Remove any empty trailing field(s)
        if ~isempty(fields) && strcmp(fields{end}, '')
            fields(end) = [];
        end
        
        % Check if the received line matches the expected number of fields
        if numel(fields) == expectedFields
            try
                % Parse the fixed fields
                raw_time   = str2double(strrep(strtrim(fields{1}), "time:", ""));
                absoluteTimeSec = raw_time;
                Ix_plus_val  = str2double(strrep(strtrim(fields{2}), "Ix_plus:", ""));
                Ix_minus_val = str2double(strrep(strtrim(fields{3}), "Ix_minus:", ""));
                Iy_plus_val  = str2double(strrep(strtrim(fields{4}), "Iy_plus:", ""));
                Iy_minus_val = str2double(strrep(strtrim(fields{5}), "Iy_minus:", ""));
            catch
                warning("Error parsing fixed fields in line: %s", dataLine);
                continue;
            end
            
            % Record the first sample as reference time
            if ~startTimeSet
                startTime = absoluteTimeSec;
                startTimeSet = true;
            end
            
            % Compute the relative time (starting from 0)
            relativeTime = absoluteTimeSec - startTime;
            
            % Stop data collection if the desired duration is reached
            if relativeTime > readDuration
                disp("Desired sensor time duration reached. Stopping data collection.");
                break;
            end
            
            % Append the current values to vectors
            t(end+1,1) = relativeTime;
            Ix_plus(end+1,1)  = Ix_plus_val;
            Ix_minus(end+1,1) = Ix_minus_val;
            Iy_plus(end+1,1)  = Iy_plus_val;
            Iy_minus(end+1,1) = Iy_minus_val;
            
            % Parse sensor fields dynamically for each sensor
            for sensor = 1:numSensors
                % Compute the index for the current sensor's bx value.
                % Fixed fields occupy indices 1-5.
                baseIndex = 5 + 3*(sensor-1) + 1;
                sensorLabel = num2str(sensor-1);  % Sensor numbering starts at 0
                try
                    bx_val = str2double(strrep(strtrim(fields{baseIndex}),   ['bx' sensorLabel ':'], ""));
                    by_val = str2double(strrep(strtrim(fields{baseIndex+1}), ['by' sensorLabel ':'], ""));
                    bz_val = str2double(strrep(strtrim(fields{baseIndex+2}), ['bz' sensorLabel ':'], ""));
                catch
                    warning("Error parsing sensor %s fields in line: %s", sensorLabel, dataLine);
                    continue;
                end
                sensorData{sensor}.bx(end+1,1) = bx_val;
                sensorData{sensor}.by(end+1,1) = by_val;
                sensorData{sensor}.bz(end+1,1) = bz_val;
            end
        else
            warning("Unexpected data format: %s", dataLine);
        end
    end
    pause(0.00001);  % Small pause to avoid overloading the CPU
end

%% Baseline Correction for Sensor Data
for sensor = 1:numSensors
    % Use the first 200 samples (or fewer if not available) to compute the baseline
    nBaseline = min(200, length(sensorData{sensor}.bx));
    sensorData{sensor}.bx = sensorData{sensor}.bx - mean(sensorData{sensor}.bx(1:nBaseline));
    
    nBaseline = min(200, length(sensorData{sensor}.by));
    sensorData{sensor}.by = sensorData{sensor}.by - mean(sensorData{sensor}.by(1:nBaseline));
    
    nBaseline = min(200, length(sensorData{sensor}.bz));
    sensorData{sensor}.bz = sensorData{sensor}.bz - mean(sensorData{sensor}.bz(1:nBaseline));
end

%% Organize Data into Final Struct
data.t = t;
data.u.Ix_plus  = Ix_plus;
data.u.Ix_minus = Ix_minus;
data.u.Iy_plus  = Iy_plus;
data.u.Iy_minus = Iy_minus;
data.sensorData = sensorData;

disp("Final data struct:");
disp(data);

%% Plot Magnetic Field Data for Each Sensor
figure(1);
clf;
for sensor = 1:numSensors
    sensorLabel = num2str(sensor-1);
    
    % Plot bx for current sensor
    subplot(numSensors, 3, (sensor-1)*3 + 1);
    plot(data.t, sensorData{sensor}.bx);
    title(['Sensor ' sensorLabel ' bx']);
    xlabel('Time (s)');
    ylabel('Magnetic Field');
    
    % Plot by for current sensor
    subplot(numSensors, 3, (sensor-1)*3 + 2);
    plot(data.t, sensorData{sensor}.by);
    title(['Sensor ' sensorLabel ' by']);
    xlabel('Time (s)');
    ylabel('Magnetic Field');
    
    % Plot bz for current sensor
    subplot(numSensors, 3, (sensor-1)*3 + 3);
    plot(data.t, sensorData{sensor}.bz);
    title(['Sensor ' sensorLabel ' bz']);
    xlabel('Time (s)');
    ylabel('Magnetic Field');
end
sgtitle('Magnetic Field Measurements from Sensors');

%% Plot Measured Currents in Different Subplots
figure(2);
clf;
subplot(2,2,1);
plot(data.t, data.u.Ix_plus, '-o');
title('Ix plus');
xlabel('Time (s)');
ylabel('Current');

subplot(2,2,2);
plot(data.t, data.u.Ix_minus, '-o');
title('Ix minus');
xlabel('Time (s)');
ylabel('Current');

subplot(2,2,3);
plot(data.t, data.u.Iy_plus, '-o');
title('Iy plus');
xlabel('Time (s)');
ylabel('Current');

subplot(2,2,4);
plot(data.t, data.u.Iy_minus, '-o');
title('Iy minus');
xlabel('Time (s)');
ylabel('Current');

sgtitle('Measured Currents');

%% Clean Up
clear s;
