
'''
    API for scripts that want to import this module:
        get_position() -> dictionary with keys_domain = {'x', 'y', 'z', 'dx', 'dy', 'dz'} ; return None if some errors occur
        read_currents() -> dictionary with keys_domain = {'x+', 'x-', 'y+', 'y-'} ; return None if some errors occur
        set_solenoid_currents(currents : dict) -> True: correctly set ; False: errors occured during setting
        reset_solenoids() -> True: correctly set ; False: errors occured during resetting
        get_status() -> dictionary with keys_domain = {'response', 'solenoids', 'sensors', 'uptime'} ; return None if some errors occur
'''




import serial
import logging
from typing import Optional, Dict, Any, List

from cobs import *
import struct

CMD_READ_SENSORS = b'\x01'
CMD_READ_CURRENTS = b'\x02'
CMD_SET_CURRENTS = b'\x03'
CMD_RESET_CURRENTS = b'\x04'
CMD_GET_STATUS = b'\x05'

maggy_logger = logging.getLogger(__name__)

# interface for communicating with a Maglev system via serial connection
class MaglevInterface:

    # initialize the serial connection
    def __init__(self, port: str = "COM7", baud_rate: int = 115200, timeout: int = 1):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None

    # enables use of 'with' statement for automatic resource management. Propagates exceptions thrown by _open_comm()
    def __enter__(self):
        try:
            self._open_comm()
            return self
        except:
            raise

    # ensures the serial connection is closed after use. Propagates exceptions thrown by _close_comm()
    def __exit__(self, exc_type, exc_value, traceback):
        try:
            self._close_comm()
        except:
            raise

    #
    #   === private methods ===
    #

    # opens serial communication. Raises an exception on failure
    def _open_comm(self) -> None:
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
            maggy_logger.info(f"Serial connection established on {self.port} port.")
        except serial.SerialException as e:
            maggy_logger.error(f"Serial connection failed on {self.port} port.")
            raise RuntimeError(f"{e}")

    # closes serial communication. Raises an exception on failure
    def _close_comm(self) -> None:
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                maggy_logger.info(f"Serial connection closed on {self.port} port.")
            except serial.SerialException as e:
                maggy_logger.error(f"Serial connection closing on {self.port} port failed.")
                raise RuntimeError(f"{e}")

    # sends cobs encoded command and retrieves response, handling device errors, raising SerialException
    def _send_command(self, cmd_name: bytes, parameters: Optional[Dict[str, Any]] = None) -> Optional[bytes]:
        if not self.ser: # if the self.ser object has not been initialized
            maggy_logger.error("Something went wrong with serial object creation.")
            return None
        if not self.ser.is_open:
            maggy_logger.error("Serial connection is not open.")
            return None

        self.ser.reset_input_buffer()  # Prevent stale data

        parameters = parameters or {}  # ensure default empty dictionary

        command_map = {
            CMD_READ_SENSORS: CMD_READ_SENSORS,  # READ_SENSORS
            CMD_READ_CURRENTS: CMD_READ_CURRENTS,  # READ_CURRENTS
            CMD_SET_CURRENTS: bytes([0x03, parameters.get("x+", 0), parameters.get("y-", 0), parameters.get("x-", 0), parameters.get("y+", 0)]),
            CMD_RESET_CURRENTS: CMD_RESET_CURRENTS,  # RESET_CURRENTS
            CMD_GET_STATUS: CMD_GET_STATUS   # GET_STATUS
        }
        data = command_map.get(cmd_name)

        if data is None:
            return None

        enc_data = cobs_encode(data)

        try:
            self.ser.write(enc_data)
            self.ser.flush()  # Ensure data is sent

            enc_response = self.ser.read_until(b'\x00')

            if not enc_response:
                maggy_logger.error("No response received from the system.")
                return None

            # print(enc_response)

            response = cobs_decode(enc_response)

            # handle error response
            if response[0] == b'\x00':
                error_code = response[1]
                maggy_logger.error(f"Device error {error_code}.")
                return None

            return response  # successful response

        except serial.SerialException as e:
            maggy_logger.error(f"Communication failed sending/receiving commands.")
            raise RuntimeError(f"{e}")


    #
    #   === public methods ===
    #

    # gets sensor position data (X, Y, Z). Propagates exceptions thrown by _send_command()
    def get_position(self) -> Optional[Dict[str, float]]:
        try:
            response = self._send_command(CMD_READ_SENSORS)

            # print(len(response), response)

            if response is None:
                return None
            else:
                x, y, z, dx, dy, dz = struct.unpack('@ffffff', response[1:])   # thanks to '@' this code adapt to the architecture's endianness
                values = { 'x' : 0.0, 'y' : 0.0, 'z' : 0.0, 'dx' : 0.0, 'dy' : 0.0, 'dz' : 0.0 }

                values['x'] = x
                values['y'] = y
                values['z'] = z
                values['dx'] = dx
                values['dy'] = dy
                values['dz'] = dz

                return values
        except RuntimeError as e:
            raise

    # gets current values of solenoids. Propagates exceptions thrown by _send_command()
    def read_currents(self) -> Optional[Dict[str, float]]:
        try:
            response = self._send_command(CMD_READ_CURRENTS)

            if response is None:
                return None
            else:
                x, _y, _x, y = struct.unpack('@ffff', response[1:])
                currents = { 'x+' : 0.0, 'y-' : 0.0, 'x-' : 0.0, 'y+' : 0.0 }
                currents['x+'] = x
                currents['y-'] = _y
                currents['x-'] = _x
                currents['y+'] = y
                return currents
        except RuntimeError as e:
            raise

    # sets the currents for the solenoids. Propagates exceptions thrown by _send_command()
    def set_solenoid_currents(self, currents: Dict[str, int]) -> bool:
        if not all(k in currents for k in ['x+', 'y-', 'x-', 'y+']):
            maggy_logger.error("Missing keys in currents dictionary.")
            return False

        try:
            response = self._send_command(CMD_SET_CURRENTS, currents)

            if response is None:
                return False
            else:
                return True
        except RuntimeError as e:
            raise

    # resets solenoids to default values. Propagates exceptions thrown by _send_command()
    def reset_solenoids(self) -> bool:
        try:
            response = self._send_command(CMD_RESET_CURRENTS)

            if response is None:
                return False
            else:
                return True
        except RuntimeError as e:
            raise

    # retrieves system status. Propagates exceptions thrown by _send_command()
    def get_status(self) -> Optional[Dict[str, Any]]:
        try:
            response = self._send_command(CMD_GET_STATUS)

            if response is None:
                return None
            else:
                solenoids, sensors, uptime = struct.unpack('@BBH', response[1:])
                status = { 'solenoids' : 0.0, 'sensors' : 0.0, 'uptime' : 0.0 }
                status['solenoids'] = solenoids
                status['sensors'] = sensors
                status['uptime'] = uptime
                return status
        except RuntimeError as e:
            raise
