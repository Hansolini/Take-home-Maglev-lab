
import time
import sys
import logging

from maggy import *

# configure logging
logging.basicConfig(level=logging.DEBUG, format="%(asctime)s - %(levelname)s: %(message)s")

# example usage
if __name__ == "__main__":
    try:
        with MaglevInterface("COM5", 115200) as maglev:
            time.sleep(2)  # wait for Teensy to initialize

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

                if r is None:
                    print("Command failed or not found.")
                else:
                    print(r)

                print()

    except RuntimeError as e:
        print("Runtime error occured!")
        sys.exit(1)  # exit immediately if serial connection fails

    print("Serial connection closed. Exiting.")
