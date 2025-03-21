import time
import sys
from maggy import *
import numpy as np
import matplotlib.pyplot as plt

# === PD Controller Gains ===
Kp = 110.0  # Proportional gain
Kd = 0.7    # Derivative gain

# === Filter Constants ===
ALPHA = 0.2   # Low-pass filter for magnetic field
DALPHA = 0.05  # Low-pass filter for derivatives
PWMALPHA = 0.3 # 0.8

# === Control Timing ===
SENSOR_FREQUENCY = 1200.0  # Hz
CONTROL_FREQUENCY = SENSOR_FREQUENCY
#CONTROL_FREQUENCY = 1000.0  # Hz
SENSOR_INTERVAL = 1.0 / SENSOR_FREQUENCY  # seconds
CONTROL_INTERVAL = 1.0 / CONTROL_FREQUENCY  # seconds

# === System State Variables ===
mag_field = {"x": 0, "y": 0, "z": 0}
raw_mag_field = {"x": 0, "y": 0, "z": 0}
prev_mag_field = {"x": 0, "y": 0, "z": 0}
d_mag_field = {"x": 0, "y": 0}

current_values = {"X_POS": 0, "X_NEG": 0, "Y_POS": 0, "Y_NEG": 0}
pwm_control_input = {"x": 0, "y": 0}

# === Mean Calibration Offsets ===
mean_mag_field = {"x": 0, "y": 0, "z": 0}


# configure logging
logging.basicConfig(level=logging.DEBUG, format="%(asctime)s - %(levelname)s: %(message)s")


# === Sensor Calibration ===
def calibrate_sensor():
    print("Calibrating sensor...")
    total = {"x": 0, "y": 0, "z": 0}

    for _ in range(1000):
        sensor_data = maglev.get_position()
        for key in total:
            total[key] += sensor_data[key] / 1000.0
        time.sleep(0.001)  # 1ms delay

    global mean_mag_field
    mean_mag_field = total
    print("Calibration complete:", mean_mag_field)


# === Apply Control Signals ===
def apply_control_signals():
    maglev.set_solenoid_currents({
        "x+": int(max(0, pwm_control_input["x"])),
        "x-": -int(min(0, pwm_control_input["x"])),
        "y+": int(max(0, pwm_control_input["y"])),
        "y-": -int(min(0, pwm_control_input["y"])),
    })


# === Main Loop ===
def run_controller():
    calibrate_sensor()

    previous_sensor_time = time.time()
    previous_control_time = time.time()

    state_list = []
    input_list = []
    timesteps = []

    try:
        while True:
            current_time = time.time()

            # === Sensor Update ===
            if current_time - previous_sensor_time >= SENSOR_INTERVAL:
                dt = current_time - previous_sensor_time
                previous_sensor_time = current_time

                # Read sensor values
                raw_mag_field.update(maglev.get_position())

                # Read solenoid currents
                current_values.update(maglev.read_currents())

                # Adjust magnetic field values
                raw_mag_field["x"] += -mean_mag_field["x"]
                raw_mag_field["y"] += -mean_mag_field["y"]
                raw_mag_field["z"] += -mean_mag_field["z"]

                # === Filtering ===
                for axis in ["x", "y", "z"]:
                    mag_field[axis] = ALPHA * raw_mag_field[axis] + (1.0 - ALPHA) * prev_mag_field[axis]

                # Compute derivatives (rate of change)
                for axis in ["x", "y"]:
                    d_mag_field[axis] = DALPHA * ((mag_field[axis] - prev_mag_field[axis]) / dt) + (1.0 - DALPHA) * d_mag_field[axis]

                # Store previous values
                prev_mag_field.update(mag_field)

            # === Control Update ===
            if current_time - previous_control_time >= CONTROL_INTERVAL:
                previous_control_time = current_time

                # PD Controller Logic
                if abs(mag_field["z"]) > 18:
                    pwm_control_input["x"] = max(-150, min(150, Kp * -mag_field["x"] + Kd * -d_mag_field["x"]))
                    pwm_control_input["y"] = max(-150, min(150, Kp * -mag_field["y"] + Kd * -d_mag_field["y"]))

                    pwm_control_input["x"] = PWMALPHA*pwm_control_input["x"] + (1.0 - PWMALPHA)*prevPwmInputX;
                    pwm_control_input["y"] = PWMALPHA*pwm_control_input["y"] + (1.0 - PWMALPHA)*prevPwmInputY;
                else:
                    pwm_control_input["x"] = 0
                    pwm_control_input["y"] = 0

                # Apply control signals
                apply_control_signals()
                timesteps.append(current_time)
                state_list.append([mag_field["x"], mag_field["y"]])
                input_list.append([pwm_control_input["x"], pwm_control_input["y"]])


                prevPwmInputX = pwm_control_input["x"]
                prevPwmInputY = pwm_control_input["y"]

                # Log system state every 10 iterations
                if time.time() % (CONTROL_INTERVAL * 10) < CONTROL_INTERVAL:
                    print(f"MagX: {mag_field['x']:.2f}, MagY: {mag_field['y']:.2f}, MagZ: {mag_field['z']:.2f}, PWM_X: {pwm_control_input['x']}, PWM_Y: {pwm_control_input['y']}")

    except KeyboardInterrupt:
        print("\nStopping controller.")
        #maglev.reset_solenoids()
        maglev._close_comm()

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
        with MaglevInterface("COM7", 115200) as maglev:
            time.sleep(2)  # wait for Teensy to initialize
            run_controller()

    except RuntimeError as e:
        print("Runtime error occured!")
        sys.exit(1)  # exit immediately if serial connection fails
