import numpy as np
import matplotlib.pyplot as plt
import csv

# Function to read measurements from a CSV file
def read_measurements(filename):
    time = []
    theta = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            time.append(float(row[0]))  
            theta.append(float(row[1])) 
    return np.array(time), np.array(theta)

# Function to read parameters from a CSV file
def read_parameters(filename):
    parameters = {}
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            key = row[0]
            value = float(row[1])
            parameters[key] = value
    return parameters

# File paths
param_file = r'C:\Users\giuse\OneDrive\Desktop\personale\Triennale\Terzo anno\Secondo semestre\Tesi\myThesis\csv\parameters.csv'  
data_file = r'C:\Users\giuse\OneDrive\Desktop\personale\Triennale\Terzo anno\Secondo semestre\Tesi\myThesis\csv\measured_rotation.csv'    

# Read the theoretical model parameters
params = read_parameters(param_file)

# Extract parameters
A_param = params.get('A', None)
beta0 = params.get('beta0', None)
beta1 = params.get('beta1', None)
omega0 = params.get('omega0', None)
omega1 = params.get('omega1', None)
phi = params.get('phi', None)
offset = params.get('offset', 0.0)  

# Read the actual measurements
time, theta_measured_deg = read_measurements(data_file)

# Convert measurements from degrees to radians
theta_measured_rad = np.deg2rad(theta_measured_deg)

# Function for the theoretical model using parameters
def damped_oscillator_model(t, A, beta0, beta1, omega0, omega1, phi, offset):
    beta_t = beta0 + beta1 * t
    omega_t = omega0 + omega1 * t
    omega_t_phi = omega_t * t + phi
    return A * np.exp(-beta_t * t) * np.cos(omega_t_phi) + offset

# Calculate the theoretical model
theta_theoretical = damped_oscillator_model(time, A_param, beta0, beta1, omega0, omega1, phi, offset)

# Plot of the measurements and theoretical model
plt.figure(figsize=(12, 6))
plt.plot(time, theta_measured_deg, label='Measurements', color='blue')
plt.plot(time, np.rad2deg(theta_theoretical), label='Theoretical Model', linestyle='--', color='orange')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.title('Measurements vs Theoretical Model')
plt.legend()
plt.grid(True)
plt.show()

# Calculate the residuals between measurements and theoretical model
residuals = theta_measured_rad - theta_theoretical

# Estimate of sigma_z (standard deviation of measurement noise)
sigma_z = np.std(residuals)
print(f"Estimate of sigma_z (standard deviation of measurement noise): {sigma_z:.4f} rad")

# Measurement noise covariance matrix R
R = np.array([[sigma_z**2]])

# Sampling interval (since the sample distribution is constant)
dt = time[1] - time[0]  

# Initialize the state vector
theta_0 = theta_measured_rad[0]  
omega_0 = (theta_measured_rad[1] - theta_measured_rad[0]) / dt  # Approximate initial angular velocity
x = np.array([theta_0, omega_0])

# Error covariance matrix P
P = [[1, 0],[0, 1]]

# Observation matrix C, we are only observing the rotation angle
H = np.array([[1, 0]])

# Array to save the likelihood function for each sigma_q
likelihood = []
sigma_q_values = np.logspace(-7, 1, num=50) # Generate 50 values evenly spaced on a logarithmic scale between 10^(-8) and 10^0
best_likelihood = -np.inf # Initialize with the most unlikely value
best_sigma_q = None
best_theta_estimated = None

for sigma_q_current in sigma_q_values:
    # Process noise covariance matrix Q
    Q = np.array([[0, 0],
                  [0, sigma_q_current**2]])

    theta_estimated = np.zeros(len(time))
    x_est = x.copy()
    P_est = P.copy()
    innovations = np.zeros(len(time))
    S_values = np.zeros(len(time))

    log_likelihood = 0.0  # Initialize the log-likelihood function

    for k in range(len(time)):
        instant_k = time[k]

        # Calculate beta_t and omega_t at time t_k
        beta_tk = beta0 + beta1 * instant_k
        omega_tk = omega0 + omega1 * instant_k

        # State transition matrix F_k
        F_k = np.array([
            [1, dt],
            [0, 1 - beta_tk * dt]
        ])

        # Prediction
        x_pred = F_k @ x_est
        P_pred = F_k @ P_est @ F_k.T + Q

        # Measurement residual
        z_k = theta_measured_rad[k]
        y_k = z_k - (H @ x_pred)[0]  
        innovations[k] = y_k

        # Innovation covariance
        S_k = (H @ P_pred @ H.T + R)[0, 0]  # We are only observing the angle
        S_values[k] = S_k

        # Kalman gain
        K_k = (P_pred @ H.T) / S_k  

        # Update
        x_est = x_pred + K_k[0,0] * y_k 
        P_est = ([[1, 0],[0, 1]] - K_k @ H) @ P_pred

        # Save the estimated angle
        theta_estimated[k] = x_est[0]

        # Update the likelihood function
        log_likelihood += -0.5 * (np.log(2 * np.pi * S_k) + (y_k**2) / S_k)

    likelihood.append(log_likelihood)

    if log_likelihood > best_likelihood:
        best_likelihood = log_likelihood
        best_sigma_q = sigma_q_current
        best_theta_estimated = theta_estimated.copy()
        best_innovations = innovations.copy()
        best_S_values = S_values.copy()

print(f"Optimal Sigma_q (standard deviation of process noise): {best_sigma_q}")
print(f"Maximum likelihood value: {best_likelihood}")

# Convert likelihood to an array to avoid dimension problems
likelihood_array = np.array(likelihood)

# Plot of the likelihood function as a function of sigma_q
plt.figure(figsize=(10, 6))
plt.plot(sigma_q_values, likelihood_array, marker='o')
plt.xscale('log')
plt.xlabel('Sigma_q (Standard Deviation of Process Noise)')
plt.ylabel('Log-Likelihood')
plt.title('Likelihood Function vs Sigma_q')
plt.grid(True)
plt.show()

# Compare measurements and Kalman filter estimates
plt.figure(figsize=(12, 6))
plt.plot(time, theta_measured_deg, label='Measurements', color='blue')
plt.plot(time, np.rad2deg(best_theta_estimated), label='Kalman Filter Estimates', color='#999910', linestyle='--', linewidth=0.7)
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.title('Measurements vs Kalman Filter Estimates')
plt.legend()
plt.grid(True)
plt.show()

# Calculate the residuals between the Kalman filter estimates and the measurements
residuals_kalman_meauserements = best_theta_estimated - theta_measured_rad
# Residuals between Kalman filter estimates and measurements
plt.figure(figsize=(12, 6))
plt.plot(time, np.rad2deg(residuals_kalman_meauserements), label='Residuals (Estimates - Measurements)', color='#559922')
plt.xlabel('Time (s)')
plt.ylabel('Residuals (deg)')
plt.title('Residuals between Kalman Filter Estimates and Measurements')
plt.legend()
plt.grid(True)
plt.show()

# Plot of the innovations
plt.figure(figsize=(12, 6))
plt.plot(time, np.rad2deg(best_innovations), label='Innovations', color='purple')
plt.xlabel('Time (s)')
plt.ylabel('Innovation (deg)')
plt.title('Kalman Filter Innovations')
plt.legend()
plt.grid(True)
plt.show()

# Plot of Kalman filter estimates vs Theoretical Model
plt.figure(figsize=(12, 6))
plt.plot(time, np.rad2deg(best_theta_estimated), label='Kalman Filter Estimates', color='#999910', linestyle='--', linewidth=0.7)
plt.plot(time, np.rad2deg(theta_theoretical), label='Theoretical Model', color='orange', linestyle='-', linewidth=0.7)
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.title('Kalman Filter Estimates vs Theoretical Model')
plt.legend()
plt.grid(True)
plt.show()

# Calculate the residuals between the Kalman filter estimates and Theoretical Model
residuals_kalman_theoretical = best_theta_estimated - theta_theoretical

# Residuals between Kalman filter estimates and Theoretical Model
plt.figure(figsize=(12, 6))
plt.plot(time, np.rad2deg(residuals_kalman_theoretical), label='Residuals (Estimates - Theoretical Model)', color='pink')
plt.xlabel('Time (s)')
plt.ylabel('Residuals (deg)')
plt.title('Residuals between Kalman Filter Estimates and Theoretical Model')
plt.legend()
plt.grid(True)
plt.show()

# A few measurements on the innovations
mean_innovations = np.mean(best_innovations)
std_innovations = np.std(best_innovations)
print(f"Mean of innovations: {np.degrees(mean_innovations):.6f} deg")
print(f"Standard deviation of innovations: {np.degrees(std_innovations):.6f} deg")


# Calculate signal-to-noise ratio (SNR)
signal_power = np.mean(theta_measured_rad**2)
noise_power = sigma_z**2
snr = 10 * np.log10(signal_power / noise_power)
print(f"Signal-to-Noise Ratio (SNR): {snr:.2f} dB")

# Calculate RMSE between the filter estimates and the measurements
rmse_final = np.sqrt(np.mean((best_theta_estimated - theta_measured_rad)**2))
print(f"RMSE between filter estimates and measurements: {rmse_final:.6f} rad ({np.degrees(rmse_final):.6f} deg)")












