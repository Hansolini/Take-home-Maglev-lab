

'''
    !!! old version of the code, not used in the final version of the project !!!
    
        ==> in the new version, the maggy.py include a class MaglevInterface. So the code below needs to be adapted to use the class
'''


import time
import maggy

# === PD Controller Gains ===
Kp = 130.0  # Proportional gain
Kd = 0.5    # Derivative gain

# === Filter Constants ===
ALPHA = 0.10   # Low-pass filter for magnetic field
DALPHA = 0.02  # Low-pass filter for derivatives

# === Control Timing ===
SENSOR_FREQUENCY = 5000.0  # Hz
CONTROL_FREQUENCY = 1000.0  # Hz
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


# === Sensor Calibration ===
def calibrate_sensor():
    print("Calibrating sensor...")
    total = {"x": 0, "y": 0, "z": 0}

    for _ in range(1000):
        sensor_data = maggy.read_sensors()
        for key in total:
            total[key] += sensor_data[key] / 1000.0
        time.sleep(0.001)  # 1ms delay

    global mean_mag_field
    mean_mag_field = total
    print("Calibration complete:", mean_mag_field)


# === Apply Control Signals ===
def apply_control_signals():
    maggy.set_solenoid_currents({
        "X_POS": pwm_control_input["x"],
        "X_NEG": -pwm_control_input["x"],
        "Y_POS": pwm_control_input["y"],
        "Y_NEG": -pwm_control_input["y"],
    })


# === Main Loop ===
def run_controller():
    maggy.open_comm()
    calibrate_sensor()

    previous_sensor_time = time.time()
    previous_control_time = time.time()

    try:
        while True:
            current_time = time.time()

            # === Sensor Update (5 kHz) ===
            if current_time - previous_sensor_time >= SENSOR_INTERVAL:
                dt = current_time - previous_sensor_time
                previous_sensor_time = current_time

                # Read sensor values
                raw_mag_field.update(maggy.read_sensors())

                # Read solenoid currents
                current_values.update(maggy.read_currents())

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

            # === Control Update (1 kHz) ===
            if current_time - previous_control_time >= CONTROL_INTERVAL:
                previous_control_time = current_time

                # PD Controller Logic
                if abs(mag_field["z"]) > 2:
                    pwm_control_input["x"] = max(-150, min(150, Kp * -mag_field["x"] + Kd * -d_mag_field["x"]))
                    pwm_control_input["y"] = max(-150, min(150, Kp * -mag_field["y"] + Kd * -d_mag_field["y"]))
                else:
                    pwm_control_input["x"] = 0
                    pwm_control_input["y"] = 0

                # Apply control signals
                apply_control_signals()

                # Log system state every 10 iterations
                if time.time() % (CONTROL_INTERVAL * 10) < CONTROL_INTERVAL:
                    print(f"MagX: {mag_field['x']:.2f}, MagY: {mag_field['y']:.2f}, MagZ: {mag_field['z']:.2f}, PWM_X: {pwm_control_input['x']}, PWM_Y: {pwm_control_input['y']}")

    except KeyboardInterrupt:
        print("\nStopping controller.")
        maggy.reset_solenoids()
        maggy.close_comm()


# Run the controller
if __name__ == "__main__":
    run_controller()
