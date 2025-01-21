# This script performs parameter fitting for an underdamped harmonic oscillator
# It reads experimental data from a CSV file and fits the model to estimate A, beta, omega, and phi
# The script also plots the original data and the fitted curve, including the fitted parameters with units

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import csv

# Function to read measurements from the CSV file
def read_measurements(filename):
    time = []
    theta = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            time.append(float(row[0]))
            theta.append(float(row[1]))
    return np.array(time), np.array(theta)  # Data are already in radians

# Equation of motion of the damped harmonic oscillator
def damped_harmonic_oscillator(t, A, beta, omega, phi):
    return A * np.exp(-beta * t) * np.cos(omega * t + phi)

# Read data
filename = 'C:/Users/giuse/OneDrive/Desktop/personale/Triennale/Terzo anno/Secondo semestre/Tesi/myThesis/csv/ideal_damped_harmonic_oscillator_response.csv'
time, theta = read_measurements(filename)

# Perform fitting with initial estimates
popt, pcov = curve_fit(damped_harmonic_oscillator, time, theta, maxfev=100000)

# Extract optimized parameters
A_opt, beta_opt, omega_opt, phi_opt = popt

print(f"Fitted parameters: A = {A_opt}, beta = {beta_opt}, omega = {omega_opt}, phi = {np.rad2deg(phi_opt)} degrees")

# Save the parameters to a CSV file
output_filename = 'C:/Users/giuse/OneDrive/Desktop/personale/Triennale/Terzo anno/Secondo semestre/Tesi/myThesis/csv/parameters.csv'

with open(output_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['A', A_opt])
    writer.writerow(['beta', beta_opt])
    writer.writerow(['omega', omega_opt])
    writer.writerow(['phi', phi_opt])

# Calculate the fitted values using the optimized parameters
fit_values = damped_harmonic_oscillator(time, A_opt, beta_opt, omega_opt, phi_opt)

# Plot the original data and the fitted curve
plt.figure(figsize=(10, 6))
plt.plot(time, theta, 'b-', label='Original Data (rad)')
plt.plot(time, fit_values, 'orange', label='Fitted Curve (rad)')
plt.xlabel('Time (s)')
plt.ylabel('Theta (rad)')
plt.legend()
plt.grid(True)
plt.title('Damped Harmonic Oscillator: Data vs Fitted Curve')

# Add annotation with the fitted parameters and their units
annotation_text = (
    f'A = {A_opt:.2f} rad\n'
    f'$\\beta$ = {beta_opt:.2f} s$^{{-1}}$\n'
    f'$\\omega$ = {omega_opt:.2f} rad/s\n'
    f'$\\phi$ = {np.rad2deg(phi_opt):.2f}$^\\circ$'
)
plt.text(0.05, 0.95, annotation_text, transform=plt.gca().transAxes,
         fontsize=12, verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

plt.show()

# Plot the residuals (difference between data and fit)
fit_error = theta - fit_values

plt.figure(figsize=(10, 6))
plt.plot(time, fit_error, 'darkgreen', label='Fitting Error (Data - Fit) (rad)')
plt.axhline(0, color='black', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Residual (Theta in rad)')
plt.title('Difference Between Data and Fitting (in radians)')
plt.legend()
plt.grid(True)
plt.show()



