classdef TeensyComms < handle
    % TeensyComms Class handles communication with Teensy and data management
    properties
        startTime
        numSensors
        numSolenoids
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
            obj.startTime = tic;

            % Initialize the serial port
            port = obj.selectTeensyPort();
            obj.initializeSerialPort(port);
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
            % Read data from Teensy and return separated data
            try
                % Expected data size is: 1 timestamp + numSensors + numSolenoids
                expectedDataSize = 1 + obj.numSensors + obj.numSolenoids;
                rawData = com_port_operations_mex('read');  % Read bytes from Teensy
                rawData = uint8(rawData);  % Ensure it's treated as raw bytes
                data = double(reshape(typecast(rawData, 'single'), expectedDataSize, [])');  % Convert to an array of floats

                % Separate the data into timestamps, sensor data, and solenoid currents
                timeStamps = data(:, 1) / 1e6;  % Convert microseconds to seconds
                sensorData = data(:, 2:(1 + obj.numSensors));  % Extract sensor data
                solenoidCurrents = data(:, (2 + obj.numSensors):end);  % Extract solenoid currents

                % Add new data to the buffer
                obj.addDataToBuffer(data);
            catch
                warning("Connection lost. Reconnecting...");
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
                clampedData = min(max(inputData, -255), 255);

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
                timeStamps = obj.dataBuffer(:, 1) / 1e6;  % Convert microseconds to seconds
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
