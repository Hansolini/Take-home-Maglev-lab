
''' 
    Interface for communicating with a maglev system via serial connection

    API for scripts that want to import this module:
        get_position() -> dictionary with keys_domain = {'x', 'y', 'z'} ; return None if some errors occur
        read_currents() -> dictionary with keys_domain = {'X_POS', 'X_NEG', 'Y_POS', 'Y_NEG'} ; return None if some errors occur
        set_solenoid_currents(currents : dict) -> True: correctly set ; False: errors occured during setting
        reset_solenoids() -> True: correctly set ; False: errors occured during resetting
        get_status() -> dictionary with keys_domain = {'response', 'solenoids', 'sensors', 'uptime'} ; return None if some errors occur
'''

import serial
import json
import logging
from typing import Optional, Dict, Any, List

maggy_logger = logging.getLogger(__name__)

# interface for communicating with a Maglev system via serial connection
class MaglevInterface:

    MOTORS_PIN: List[tuple] = [(4, 5), (2, 3), (6, 7), (8, 9)]
    KEYS: List[str] = ["Y_NEG", "X_POS", "X_NEG", "Y_POS"]

    # initialize the serial connection
    def __init__(self, port: str = "COM5", baud_rate: int = 115200, timeout: int = 1):
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

    # sends a JSON command and retrieves response, handling device errors, raising SerialException and JSONDecodeErrors
    def _send_command(self, cmd_name: str, parameters: Optional[Dict[str, Any]] = None) -> Optional[Dict[str, Any]]:
        if not self.ser: # if the self.ser object has not been initialized
            maggy_logger.error("Something went wrong with serial object creation.")
            return None
        if not self.ser.is_open:
            maggy_logger.error("Serial connection is not open.")
            return None

        parameters = parameters or {}  # ensure default empty dictionary
        data = json.dumps({"command": cmd_name, **parameters}) + "\n"

        try:
            self.ser.write(data.encode())
            response = self.ser.readline().decode().strip()

            if not response:
                maggy_logger.error("No response received from the system.")
                return None

            response_json = json.loads(response)

            # handle error response
            if response_json.get("response") == "ERROR":
                error_code = response_json.get("code", "UNKNOWN")
                error_message = response_json.get("message", "No message provided")
                maggy_logger.error(f"Device error {error_code} ({error_message}).")
                return None

            return response_json  # successful response

        except serial.SerialException as e:
            maggy_logger.error(f"Communication failed sending/receiving commands.")
            raise RuntimeError(f"{e}")
        except json.JSONDecodeError as e:
            maggy_logger.error(f"Invalid JSON response.")
            raise RuntimeError(f"{e}")


    #
    #   === public methods ===
    #

    # gets sensor position data (X, Y, Z). Propagates exceptions thrown by _send_command()
    def get_position(self) -> Optional[Dict[str, float]]:
        try:
            response = self._send_command("READ_SENSORS")
            return response.get("values", [{}])[0] if response else None
        except RuntimeError as e:
            raise

    # gets current values of solenoids. Propagates exceptions thrown by _send_command()
    def read_currents(self) -> Optional[Dict[str, float]]:
        try:
            response = self._send_command("READ_CURRENTS")
            return response.get("values", [{}])[0] if response else None
        except RuntimeError as e:
            raise

    # sets the currents for the solenoids. Propagates exceptions thrown by _send_command()
    def set_solenoid_currents(self, currents: Dict[str, float]) -> bool:
        if not all(k in currents for k in self.KEYS):
            maggy_logger.error("Missing keys in currents dictionary.")
            return False

        solenoids = [
            {"pin1": self.MOTORS_PIN[i][0], "pin2": self.MOTORS_PIN[i][1], "current": currents[self.KEYS[i]]}
            for i in range(len(self.MOTORS_PIN))
        ]

        try:
            response = self._send_command("SET_CURRENTS", {"solenoids": solenoids})
            return response is not None and response.get("status") == "OK"
        except RuntimeError as e:
            raise

    # resets solenoids to default values. Propagates exceptions thrown by _send_command()
    def reset_solenoids(self) -> bool:
        try:
            response = self._send_command("RESET_CURRENTS")
            return response is not None and response.get("status") == "OK"
        except RuntimeError as e:
            raise

    # retrieves system status. Propagates exceptions thrown by _send_command()
    def get_status(self) -> Optional[Dict[str, Any]]:
        try:
            response = self._send_command("GET_STATUS")
            return response if response else None
        except RuntimeError as e:
            raise
