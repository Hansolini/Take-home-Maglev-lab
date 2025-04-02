import time
import sys
from maggy import *
import numpy as np
import matplotlib.pyplot as plt

# === PD Controller Gains ===
Kp = 120.0  # Proportional gain
Kd = 2.3    # Derivative gain

# === Filter Constant ===
PWMALPHA = 0.82 # 0.8

# === Control Timing ===
CONTROL_FREQUENCY = 800.0  # Hz
CONTROL_INTERVAL = 1.0 / CONTROL_FREQUENCY  # seconds

# === System State Variables ===
mag_field = {"x": 0.0, "y": 0.0, "z": 0.0}
d_mag_field = {"x": 0.0, "y": 0.0, "z": 0.0}

pwm_control_input = {"x": 0, "y": 0}


# === Logging Configuration ===
logging.basicConfig(level=logging.DEBUG, format="%(asctime)s - %(levelname)s: %(message)s")


def apply_control_signals():
    maglev.set_solenoid_currents({
        "x+": int(max(0, pwm_control_input["x"])),
        "x-": -int(min(0, pwm_control_input["x"])),
        "y+": int(max(0, pwm_control_input["y"])),
        "y-": -int(min(0, pwm_control_input["y"])),
    })


# === Main Loop ===
def run_controller():

    previous_control_time = time.time()

    # state_list = []
    # input_list = []
    # timesteps = []

    # varaibles to check actual conrtol frequency
    avg_control_time_list = 0
    control_counter = 0

    try:
        while True:
            current_time = time.time()

            # === Control Update ===
            if current_time - previous_control_time >= CONTROL_INTERVAL:
                dt2 = current_time - previous_control_time
                previous_control_time = current_time

                values = maglev.get_position()

                mag_field["x"] = values["x"]
                mag_field["y"] = values["y"]
                mag_field["z"] = values["z"]
                d_mag_field["x"] = values["dx"]
                d_mag_field["y"] = values["dy"]
                d_mag_field["z"] = values["dz"]

                # PD Controller Logic
                if abs(mag_field["z"]) > 18:
                    pwm_control_input["x"] = max(-150, min(150, Kp * -mag_field["x"] + Kd * -d_mag_field["x"]))
                    pwm_control_input["y"] = max(-150, min(150, Kp * -mag_field["y"] + Kd * -d_mag_field["y"]))

                    # print(pwm_control_input)

                    pwm_control_input["x"] = PWMALPHA*pwm_control_input["x"] + (1.0 - PWMALPHA)*prevPwmInputX;
                    pwm_control_input["y"] = PWMALPHA*pwm_control_input["y"] + (1.0 - PWMALPHA)*prevPwmInputY;
                else:
                    pwm_control_input["x"] = 0
                    pwm_control_input["y"] = 0

                # Apply control signals
                apply_control_signals()

                # timesteps.append(current_time)
                # state_list.append([mag_field["x"], mag_field["y"]])
                # input_list.append([pwm_control_input["x"], pwm_control_input["y"]])


                prevPwmInputX = pwm_control_input["x"]
                prevPwmInputY = pwm_control_input["y"]

                if time.time() % (CONTROL_INTERVAL) < CONTROL_INTERVAL:
                    avg_control_time_list += int(1/dt2)
                    control_counter += 1

    except KeyboardInterrupt:
        print("\nStopping controller.")
        # maglev.reset_solenoids()
        maglev._close_comm()


    print()
    print(f"Avarage Control Frequency: {avg_control_time_list/control_counter}")
    print()

    # states = np.array(state_list)
    # inputs = np.array(input_list)
    # timesteps = np.array(timesteps) - timesteps[0]
    # fig, ax = plt.subplots(2, 1, sharex=True, sharey=False)
    # ax[0].plot(timesteps, states[:, 0], label='x')
    # ax[0].plot(timesteps, states[:, 1], label='y')
    # ax[0].grid()
    #
    # ax[1].plot(timesteps, inputs[:, 0], label='x')
    # ax[1].plot(timesteps, inputs[:, 1], label='y')
    # ax[1].grid()
    # plt.legend()
    # plt.show()


# Run the controller
if __name__ == "__main__":
    try:
        with MaglevInterface("COM31") as maglev:
            print("Keep the magnet away from the system. Sensor calibration in progress...")
            time.sleep(2)  # wait for Teensy to initialize
            print("Calibration completed.")
            run_controller()

    except RuntimeError as e:
        print("Runtime error occured!")
        sys.exit(1)  # exit immediately if serial connection fails
