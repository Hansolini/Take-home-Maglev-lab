classdef TeensyComms < handle
    % TeensyComms Class handles communication with Teensy and data management
    properties
        startTime
        numSensors
        numSolenoids
        pwm_bit_size = 12;
    end

    properties (Access = protected)
        serialPort
        dataBuffer
        bufferLimit = 20000;    % Limit the buffer bufferLimit/5kHz ~> 4s
        meanOutput
        totalSamples = 0;
    end

    methods
        function obj = TeensyComms(numSensors, numSolenoids)
            % Constructor initializes the serial connection, data buffer, and configuration
            obj.numSensors = numSensors;
            obj.numSolenoids = numSolenoids;
            obj.meanOutput = zeros(1, numSensors);
            obj.dataBuffer = zeros(obj.bufferLimit, 1 + numSensors + numSolenoids);

            % Initialize the serial port
            port = obj.selectTeensyPort();
            obj.initializeSerialPort(port);

            obj.startTime = tic;
        end

        function delete(obj)
            % Destructor to clean up resources
            obj.closeSerialPort();
            disp('Serial port closed and object deleted.');
        end

        function port = selectTeensyPort(obj)
            % Lists available ports and prompts user to select
            ports = serialportlist("available");
            
            if isempty(ports)
                error('No available serial ports detected.');
            end
            
            % Display available ports
            disp('Available USB Ports:');
            for i = 1:length(ports)
                fprintf('%d: %s\n', i, ports{i});
            end
            
            % Prompt the user to select a port
            choice = input('Select the port number corresponding to the Teensy device: ');
            
            % Validate the user's choice
            if choice < 1 || choice > length(ports)
                error('Invalid selection. Please run the script again and select a valid port number.');
            end
            
            port = ports{choice};
            disp(['Selected port: ', port]);
        end

        function initializeSerialPort(obj, port)
            % Initialize serial port communication
            obj.serialPort = port;
            com_port_operations_mex('open', port);
            com_port_operations_mex('set_timeout');
        end

        function closeSerialPort(obj)
            % Close the serial port
            com_port_operations_mex('close');
        end
        
        function [timeStamps, sensorData, solenoidCurrents] = readData(obj)
            try
                % Define the sizes of each type in bytes
                bytesPerTimestamp = 4; % unsigned long (4 bytes)
                bytesPerSensorValue = 4; % float (4 bytes)
                bytesPerSolenoidCurrent = 2; % int16_t (2 bytes)
        
                % Calculate the total bytes expected per sample
                totalBytesPerSample = bytesPerTimestamp + ...
                    (obj.numSensors * bytesPerSensorValue) + ...
                    (obj.numSolenoids * bytesPerSolenoidCurrent);
        
                % Read raw byte data from Teensy
                rawData = com_port_operations_mex('read');  % Read bytes from Teensy
                rawData = uint8(rawData);  % Ensure it's treated as raw bytes
        
                % Check if received data size matches an expected multiple of totalBytesPerSample
                numBytesReceived = length(rawData);
                if mod(numBytesReceived, totalBytesPerSample) ~= 0
                    warning('Received data size does not match expected sample size.');
                    timeStamps = [];
                    sensorData = [];
                    solenoidCurrents = [];
                    return;
                end
        
                % Calculate the number of samples based on received data size
                numSamples = numBytesReceived / totalBytesPerSample;
        
                % Reshape the raw data into rows corresponding to each sample
                reshapedData = reshape(rawData, totalBytesPerSample, numSamples)';
        
                % Extract and convert timestamps (first 4 bytes as unsigned long)
                timeStamps = double(typecast(reshape(reshapedData(:, 1:bytesPerTimestamp)', 1, []), 'uint32'))' / 1e6;
        
                % Extract and convert sensor data (next bytes as float)
                sensorData = zeros(numSamples, obj.numSensors);
                for i = 1:obj.numSensors
                    startIdx = bytesPerTimestamp + (i-1) * bytesPerSensorValue + 1;
                    endIdx = startIdx + bytesPerSensorValue - 1;
                    sensorData(:, i) = typecast(reshape(reshapedData(:, startIdx:endIdx)', 1, []), 'single')';
                end
        
                % Extract and convert solenoid currents (remaining bytes as int16)
                solenoidCurrents = zeros(numSamples, obj.numSolenoids);
                for i = 1:obj.numSolenoids
                    startIdx = bytesPerTimestamp + obj.numSensors * bytesPerSensorValue + (i-1) * bytesPerSolenoidCurrent + 1;
                    endIdx = startIdx + bytesPerSolenoidCurrent - 1;
                    solenoidCurrents(:, i) = typecast(reshape(reshapedData(:, startIdx:endIdx)', 1, []), 'int16')';
                end
        
                % Add new data to the buffer
                obj.addDataToBuffer([timeStamps, sensorData, solenoidCurrents]);
            catch ME
                % Handle exceptions and provide diagnostics
                warning("Connection lost. Reconnecting...");
                disp(ME.message);
                timeStamps = [];
                sensorData = [];
                solenoidCurrents = [];
            end
        end

        function addDataToBuffer(obj, newData)
            % Add new data to the buffer, maintaining the buffer size limit
            newDataCount = size(newData, 1);
            currentBufferSize = size(obj.dataBuffer, 1);

            % Check if adding new data exceeds the buffer limit
            if currentBufferSize + newDataCount > obj.bufferLimit
                % Calculate how many rows to remove
                rowsToRemove = currentBufferSize + newDataCount - obj.bufferLimit;
                % Remove the oldest data and append new data
                obj.dataBuffer = [obj.dataBuffer(rowsToRemove+1:end, :); newData];
            else
                % Append new data without exceeding buffer limit
                obj.dataBuffer = [obj.dataBuffer; newData];
            end
        end

        function sendInputToTeensy(obj, inputData)
            % Sends input data to Teensy
            % Ensure data is in int16 format and within the range [-255, 255]
            try
                % Clamp the input data to the allowed range
                clampedData = min(max(inputData, -0.29*2^obj.pwm_bit_size), 0.29*2^obj.pwm_bit_size); % 0.3 depends on the sensing resistor in the system motor drivers (max current)!

                % Convert data to int16
                data = typecast(int16(clampedData), 'uint8');

                % Send the data to the Teensy
                com_port_operations_mex('write', data);
            catch
                warning("Failed to send data to Teensy.");
            end
        end

        function [timeStamps, sensorData, solenoidCurrents] = getBufferedData(obj)
            % Retrieve the entire time series data from the buffer
            if isempty(obj.dataBuffer)
                timeStamps = [];
                sensorData = [];
                solenoidCurrents = [];
            else
                timeStamps = obj.dataBuffer(:, 1);  % Convert microseconds to seconds
                sensorData = obj.dataBuffer(:, 2:(1 + obj.numSensors));  % Extract sensor data
                solenoidCurrents = obj.dataBuffer(:, (2 + obj.numSensors):end);  % Extract solenoid currents
            end
        end

        function updateMeans(obj, newData)
            % Update running mean of measured output data
            n = size(newData, 1);  % Number of new samples
            obj.totalSamples = obj.totalSamples + n;  % Update total sample count

            % Compute running mean for each sensor
            obj.meanOutput = ((obj.totalSamples - n) * obj.meanOutput + sum(newData, 1)) / obj.totalSamples;
        end

        function meanOutput = getMeanOutput(obj)
            % Get the current mean of the measured output data
            meanOutput = obj.meanOutput;
        end
    end
end
