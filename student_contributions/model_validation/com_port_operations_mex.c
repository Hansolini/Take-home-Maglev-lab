#include "mex.h"
#include <windows.h>
#include <stdio.h>
#include <stdint.h>  // Include stdint.h for uint8_t

HANDLE hComm = INVALID_HANDLE_VALUE;

// Function to open the COM port
void open_com_port(const char *port_name) {
    hComm = CreateFileA(port_name, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
    if (hComm == INVALID_HANDLE_VALUE) {
        mexErrMsgIdAndTxt("MATLAB:com_port_operations_mex:FailedToOpenPort", "Failed to open COM port");
    }
}

// Function to set COM port timeouts
void set_com_timeouts() {
    if (hComm == INVALID_HANDLE_VALUE) {
        mexErrMsgIdAndTxt("MATLAB:com_port_operations_mex:PortNotOpen", "COM port not open");
    }

    // Set timeouts to minimal values
    COMMTIMEOUTS timeouts = {MAXDWORD, 0, 0, 0, 0};
    if (!SetCommTimeouts(hComm, &timeouts)) {
        CloseHandle(hComm);
        mexErrMsgIdAndTxt("MATLAB:com_port_operations_mex:FailedToSetTimeouts", "Error setting COM timeouts");
    }
}

// Function to read all available data from the COM port
void read_com_port(mxArray *plhs[]) {
    if (hComm == INVALID_HANDLE_VALUE) {
        mexErrMsgIdAndTxt("MATLAB:com_port_operations_mex:PortNotOpen", "COM port not open");
    }

    // Check how many bytes are available to read
    DWORD errors;
    COMSTAT status;
    ClearCommError(hComm, &errors, &status);
    DWORD num_bytes_available = status.cbInQue;

    if (num_bytes_available > 0) {
        char *buffer = (char *)mxCalloc(num_bytes_available, sizeof(char));
        DWORD bytesRead = 0;
        BOOL result = ReadFile(hComm, buffer, num_bytes_available, &bytesRead, NULL);
        if (!result || bytesRead == 0) {
            mxFree(buffer);
            mexErrMsgIdAndTxt("MATLAB:com_port_operations_mex:ReadFailed", "Failed to read from COM port");
        }

        // Create MATLAB array to return the data
        plhs[0] = mxCreateNumericMatrix(1, bytesRead, mxUINT8_CLASS, mxREAL);
        memcpy(mxGetData(plhs[0]), buffer, bytesRead);

        mxFree(buffer);
    } else {
        // Return an empty array if no data is available
        plhs[0] = mxCreateNumericMatrix(0, 0, mxUINT8_CLASS, mxREAL);
    }
}

// Function to write data to the COM port
void write_data(const mxArray *data) {
    if (hComm == INVALID_HANDLE_VALUE) {
        mexErrMsgIdAndTxt("MATLAB:com_port_operations_mex:PortNotOpen", "COM port not open");
    }

    // Ensure data is in uint8 format
    if (!mxIsUint8(data)) {
        mexErrMsgIdAndTxt("MATLAB:com_port_operations_mex:InvalidInputType", "Data must be of type uint8.");
    }

    // Get the data to write
    uint8_t *buffer = (uint8_t *)mxGetData(data);
    size_t data_size = mxGetNumberOfElements(data);

    // Add start and end markers
    uint8_t *data_to_write = (uint8_t *)mxCalloc(data_size + 2, sizeof(uint8_t));
    data_to_write[0] = '<'; // Start marker
    memcpy(data_to_write + 1, buffer, data_size);
    data_to_write[data_size + 1] = '>'; // End marker

    // Purge any remaining data in the buffer before sending new data
    PurgeComm(hComm, PURGE_TXCLEAR | PURGE_RXCLEAR);

    // Write the data
    DWORD bytes_written;
    BOOL result = WriteFile(hComm, data_to_write, (DWORD)(data_size + 2), &bytes_written, NULL);
    if (!result || bytes_written != data_size + 2) {
        mxFree(data_to_write);
        mexErrMsgIdAndTxt("MATLAB:com_port_operations_mex:WriteFailed", "Failed to write to COM port");
    }

    // Flush the output buffer to ensure immediate transmission
    if (!FlushFileBuffers(hComm)) {
        mxFree(data_to_write);
        mexErrMsgIdAndTxt("MATLAB:com_port_operations_mex:FlushFailed", "Failed to flush COM port buffers");
    }

    // Free the allocated memory
    mxFree(data_to_write);
}

// Function to close the COM port
void close_com_port() {
    if (hComm != INVALID_HANDLE_VALUE) {
        CloseHandle(hComm);
        hComm = INVALID_HANDLE_VALUE;
    }
}

// MEX function entry point
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    if (nrhs < 1) {
        mexErrMsgIdAndTxt("MATLAB:com_port_operations_mex:InvalidNumInputs", "At least one input required (command).");
    }

    char *command = mxArrayToString(prhs[0]);

    if (strcmp(command, "open") == 0) {
        if (nrhs != 2) {
            mexErrMsgIdAndTxt("MATLAB:com_port_operations_mex:InvalidNumInputs", "Port name required for 'open' command.");
        }
        char *port_name = mxArrayToString(prhs[1]);
        open_com_port(port_name);
        mxFree(port_name);
    } else if (strcmp(command, "set_timeout") == 0) {
        set_com_timeouts();
    } else if (strcmp(command, "read") == 0) {
        read_com_port(plhs);
    } else if (strcmp(command, "write") == 0) {
        if (nrhs != 2) {
            mexErrMsgIdAndTxt("MATLAB:com_port_operations_mex:InvalidNumInputs", "Data required for 'write' command.");
        }
        write_data(prhs[1]);
    } else if (strcmp(command, "close") == 0) {
        close_com_port();
    } else {
        mexErrMsgIdAndTxt("MATLAB:com_port_operations_mex:InvalidCommand", "Invalid command.");
    }

    mxFree(command);
}
