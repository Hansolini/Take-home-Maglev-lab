# This script performs parameter fitting for an underdamped harmonic oscillator
# It reads experimental data from a CSV file and fits the model to estimate A, beta, omega, and phi
# The script also plots the original data and the fitted curve, including the fitted parameters with units

import numpy as np
import matplotlib.pyplot as plt
import csv
from scipy.optimize import curve_fit

# Function to read data from the CSV file
def read_measurements(filename):
    time = []
    theta = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            time.append(float(row[0]))
            theta.append(float(row[1]))
    return np.array(time), np.array(theta)

# Damped harmonic oscillator model with constant parameters]
def damped_oscillator(t, A, beta, omega, phi):
    # gamma and omega are constants
    return A * np.exp(-beta * t) * np.cos(omega * t + phi)


# Read the original dataset
filename = r'C:\Users\giuse\OneDrive\Desktop\personale\Triennale\Terzo anno\Secondo semestre\Tesi\myThesis\csv\measured_rotation.csv'  
time, theta_deg = read_measurements(filename)

# Convert theta from degrees to radians
theta = np.deg2rad(theta_deg)


# Perform the parameter fit using curve_fit
popt, pcov = curve_fit(
    damped_oscillator,
    time,
    theta,
    maxfev=200000
)

# Extract optimal parameters
A_opt, beta_opt, omega_opt, phi_opt = popt

# Calculate the fitted values
fit_values_rad = damped_oscillator(time, *popt)

# Save the parameters in a CSV file
param_filename = r'C:\Users\giuse\OneDrive\Desktop\personale\Triennale\Terzo anno\Secondo semestre\Tesi\myThesis\csv\parameters_naive.csv' 
with open(param_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['A', A_opt])
    writer.writerow(['beta', beta_opt])
    writer.writerow(['omega', omega_opt])
    writer.writerow(['phi', phi_opt])

fit_values_deg = np.rad2deg(fit_values_rad)

# Calculate the residuals (difference between original data and fit)
residuals = theta_deg - fit_values_deg

# Calculate the sum of squared residuals (SSR)
SSR = np.sum(residuals ** 2)

# Print the sum of squared residuals
print(f'Sum of squared residuals (SSR): {SSR:.5f}')

# Print the optimal parameters
print('Optimal parameters:')
print(f'A = {A_opt}')
print(f'beta = {beta_opt}')
print(f'omega = {omega_opt}')
print(f'phi = {phi_opt}')


# Plot the original data and the fit
plt.figure(figsize=(12, 6))
plt.plot(time, theta_deg, label='Original Data')
plt.plot(time, fit_values_deg, label='Fit', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Theta (deg)')
plt.legend()
plt.title('Comparison between Original Data and Fit')
plt.grid(True)
# Add annotation with the fitted parameters and their units
annotation_text = (
    f'A = {A_opt:.2f} rad = {np.rad2deg(A_opt):.2f}$^\\circ$\n'
    f'$\\beta$ = {beta_opt:.2f} s$^{{-1}}$\n'
    f'$\\omega$ = {omega_opt:.2f} rad/s={np.rad2deg(omega_opt):.2f}$^\\circ/s$\n'
    f'$\\phi$ = {phi_opt:.2}rad={np.rad2deg(phi_opt):.2f}$^\\circ$'
)
plt.text(0.77, 0.85, annotation_text, transform=plt.gca().transAxes,
         fontsize=12, verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

plt.show()

# Plot of the residuals with SSR in the title
plt.figure(figsize=(12, 6))
plt.plot(time, residuals, label='Residuals', color='darkgreen') 
plt.xlabel('Time (s)')
plt.ylabel('Residuals (deg)')
plt.title(f'Residuals between Original Data and Fit')
plt.axhline(0, color='black', linestyle='--', linewidth=1)
plt.grid(True)
plt.legend()
plt.show()