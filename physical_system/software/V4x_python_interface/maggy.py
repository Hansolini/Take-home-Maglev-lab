
''' 
    Interface for communicating with a maglev system via serial connection
'''

import serial
import time
import sys
import json
import logging
from typing import Optional, Dict, Any, List

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class MaglevInterface:
    MOTORS_PIN: List[tuple] = [(4, 5), (2, 3), (6, 7), (8, 9)]
    KEYS: List[str] = ["Y_NEG", "X_POS", "X_NEG", "Y_POS"]

    def __init__(self, port: str = "COM5", baud_rate: int = 115200, timeout: int = 1):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None

    def __enter__(self):
        self.open_comm()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close_comm()

    # opens serial communication. Raises an exception on failure
    def _open_comm(self) -> None:
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
            logger.info("Serial connection established.")
        except serial.SerialException as e:
            logger.error(f"Serial connection failed: {e}")
            raise RuntimeError(f"Unable to open serial port {self.port}: {e}")

    # closes serial communication. Raises an exception on failure
    def _close_comm(self) -> None:
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                logger.info("Serial connection closed.")
            except serial.SerialException as e:
                logger.error(f"Error closing serial connection: {e}")
                raise RuntimeError(f"Failed to close serial connection on {self.port}: {e}")

    # sends a JSON command and retrieves response, handling errors
    def _send_command(self, cmd_name: str, parameters: Optional[Dict[str, Any]] = None) -> Optional[Dict[str, Any]]:
        if not self.ser or not self.ser.is_open:
            logger.error("Serial connection is not open.")
            return None

        parameters = parameters or {}  # Ensure default empty dictionary
        data = json.dumps({"command": cmd_name, **parameters}) + "\n"

        try:
            self.ser.write(data.encode())
            response = self.ser.readline().decode().strip()

            if not response:
                logger.error("No response received from the system.")
                return None

            response_json = json.loads(response)

            # Handle error response
            if response_json.get("response") == "ERROR":
                error_code = response_json.get("code", "UNKNOWN")
                error_message = response_json.get("message", "No message provided")
                logger.error(f"Device error {error_code}: {error_message}")
                return None  # Or raise an exception if preferred

            return response_json  # Successful response

        except serial.SerialException as e:
            logger.error(f"Communication error: {e}")
        except json.JSONDecodeError as e:
            logger.error(f"Invalid JSON response: {e}")

        return None

    # gets sensor position data (X, Y, Z)
    def get_position(self) -> Optional[Dict[str, float]]:
        response = self._send_command("READ_SENSORS")
        return response.get("values", [{}])[0] if response else None


    # gets current values of solenoids
    def read_currents(self) -> Optional[Dict[str, float]]:
        response = self._send_command("READ_CURRENTS")
        return response.get("values", [{}])[0] if response else None

    # sets the currents for the solenoids
    def set_solenoid_currents(self, currents: Dict[str, float]) -> bool:
        if not all(k in currents for k in self.KEYS):
            logger.error("Missing keys in currents dictionary.")
            return False

        solenoids = [
            {"pin1": self.MOTORS_PIN[i][0], "pin2": self.MOTORS_PIN[i][1], "current": currents[self.KEYS[i]]}
            for i in range(len(self.MOTORS_PIN))
        ]

        response = self._send_command("SET_CURRENTS", {"solenoids": solenoids})
        return response is not None and response.get("status") == "OK"

    # resets solenoids to default values
    def reset_solenoids(self) -> bool:
        response = self._send_command("RESET_CURRENTS")
        return response is not None and response.get("status") == "OK"

    # retrieves system status
    def get_status(self) -> Optional[Dict[str, Any]]:
        return self._send_command("GET_STATUS")


# example Usage
if __name__ == "__main__":
    try:
        with MaglevInterface("COM5", 115200) as maglev:
            maglev.open_comm()
            time.sleep(2)  # wait for system to initialize

            while True:
                user = input("""
                Options:
                1. Read sensors
                2. Read currents
                3. Set currents
                4. Get status
                5. Reset solenoids
                6. Quit
                \n> """)

                print()

                if user == "1":
                    r = maglev.get_position()
                elif user == "2":
                    r = maglev.read_currents()
                elif user == "3":
                    try:
                        currents = list(map(float, input("Enter 4 values separated by spaces (e.g. 0 0 0 0): ").split()))
                        if len(currents) != 4:
                            raise ValueError("Please enter exactly 4 values.")
                        dict_currents = {
                            "Y_NEG": currents[0],
                            "X_POS": currents[1],
                            "X_NEG": currents[2],
                            "Y_POS": currents[3]
                        }
                        r = maglev.set_solenoid_currents(dict_currents)
                    except ValueError as e:
                        print(f"Invalid input: {e}")
                        r = None
                elif user == "4":
                    r = maglev.get_status()
                elif user == "5":
                    r = maglev.reset_solenoids()
                elif user == "6":
                    break
                else:
                    r = None

                if r is None:
                    print("Command failed or not found.")
                else:
                    print(r)

                print()

    except RuntimeError as e:
        print(f"Error: {e}")
        sys.exit(1)

    print("Serial connection closed. Exiting.")
