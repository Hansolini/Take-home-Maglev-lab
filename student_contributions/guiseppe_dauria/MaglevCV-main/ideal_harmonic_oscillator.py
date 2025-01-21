# This program simulates the behavior of the rotation angle in a damped harmonic oscillator, based on its parameters.

import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import expm
import csv

# Realistic physical parameters
I = 1   # moment of inertia (kg·m²)
k_rot = 15  # torsional spring constant (N·m/rad)
c_rot = 1   # torsional damping coefficient (N·m·s/rad)

# Compute alpha and beta
alpha = k_rot / I
beta = c_rot / I

# Convert theta0 and omega0 from radians to degrees
theta_0_rad = np.pi / 4  # initial angle (rad)
omega_0_rad = np.pi  # initial angular velocity (rad/s)

theta_0_deg = np.degrees(theta_0_rad)  # initial angle (deg)
omega_0_deg = np.degrees(omega_0_rad)  # initial angular velocity (deg/s)

# Matrix A, with alpha and beta 
A = np.array([[0, 1],
              [-alpha, -beta]])

# Matrix C (only the angle θ)
C = np.array([[1, 0]])

# Initial conditions x(0) = [theta_0, omega_0]
x0 = np.array([[theta_0_rad], [omega_0_rad]])

time_values = np.linspace(0, 10, 1000)  # time from 0 to 10 seconds with 1000 points

# Initialization of the response vector size
n = len(time_values)
theta = np.zeros(n)

# Calculation of the time response
for i in range(n):
    t = time_values[i] 
    
    # Compute exp(A * t)
    exp_At = expm(A * t)
    
    # Compute C @ exp(A*t) 
    temp_result = np.dot(C, exp_At)  
    theta[i] = np.dot(temp_result, x0).item() 

# Save the results in a CSV
# Utilizza forward slashes (/) per evitare problemi con i backslash in Windows
csv_filename = 'C:/Users/giuse/OneDrive/Desktop/personale/Triennale/Terzo anno/Secondo semestre/Tesi/myThesis/csv/ideal_damped_harmonic_oscillator_response.csv'

# Open the file in write mode
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    for t, th in zip(time_values, theta):
        writer.writerow([t, th])

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(time_values, theta, label=r'$C \cdot e^{At} \cdot x_0$', color='b')
plt.title('Response of a Rotational Harmonic Oscillator')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.grid(True)
plt.legend()

# Prepare the annotation text with alpha, beta, theta0, omega0
# Utilizza stringhe normali e backslash doppiati per i comandi LaTeX
annotation_text = (
    '$\\alpha$ = {:.2f} rad/s²\n'
    '$\\beta$ = {:.2f} s⁻¹\n'
    '$\\theta_0$ = {:.2f}°\n'
    '$\\omega_0$ = {:.2f}°/s'.format(alpha, beta, theta_0_deg, omega_0_deg)
)

# Add the annotation text to the plot
plt.text(0.809, 0.90, annotation_text, transform=plt.gca().transAxes,
         fontsize=12, verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

plt.show()



