# Questo script stima i parametri dell'oscillatore armonico basandosi sulle misure
# Consideriamo un oscillatore con parametri variabili per migliorare la qualità del fit

import numpy as np
import matplotlib.pyplot as plt
import csv
from scipy.optimize import curve_fit
from scipy.signal import find_peaks

# Funzione per leggere i dati dal file CSV
def read_measurements(filename):
    time = []
    theta = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            time.append(float(row[0]))
            theta.append(float(row[1]))
    return np.array(time), np.array(theta)

# Modello dell'oscillatore smorzato con parametri variabili nel tempo
def damped_oscillator_variable(t, A, beta0, beta1, omega0, omega1, phi, offset):
    beta = beta0 + beta1 * t
    omega = omega0 + omega1 * t  # omega in rad/s
    omega_t = omega * t + phi    # omega * t + phi in rad
    return A * np.exp(-beta * t) * np.cos(omega_t) + offset

# Funzione per calcolare la stima iniziale dei parametri
def compute_initial_guess(time, theta):
    # Trova i picchi nei dati
    peaks_indices, _ = find_peaks(theta)
    peaks_time = time[peaks_indices]
    peaks_theta = theta[peaks_indices]

    # Stima di beta0 usando il decremento logaritmico
    A1 = peaks_theta[0]
    A2 = peaks_theta[1]
    delta = np.log(np.abs(A1 / A2))
    T_start = peaks_time[1] - peaks_time[0]
    beta0_guess = delta / T_start

    # Stima di omega0
    omega0_guess = 2 * np.pi / T_start

    # Stima dell'offset
    offset_guess = np.mean(theta)

    # Stima dell'ampiezza
    A_guess = (np.max(theta) - np.min(theta)) / 2

    # Stima della fase iniziale
    if A_guess != 0:
        phi_guess = np.arccos((theta[0] - offset_guess) / A_guess)
    else:
        phi_guess = 0

    # Stima di beta1 e omega1
    A_start = peaks_theta[0]
    A_end = peaks_theta[-1]
    delta_total = np.log(np.abs(A_start / A_end))
    T_total = peaks_time[-1] - peaks_time[0]  # Tempo totale T
    beta_mean = delta_total / T_total  # beta_mean è la media integrale di beta
    beta1_guess = 2 * (beta_mean - beta0_guess) / T_total  # Stima di beta1

    T_end = peaks_time[-1] - peaks_time[-2]  # Periodo dell'ultima oscillazione
    omega_end = 2 * np.pi / T_end
    omega1_guess = (omega_end - omega0_guess) / T_total

    initial_guess = [A_guess, beta0_guess, beta1_guess, omega0_guess, omega1_guess, phi_guess, offset_guess]
    return initial_guess

# Leggi il dataset originale
filename = r'C:\Users\giuse\OneDrive\Desktop\personale\Triennale\Terzo anno\Secondo semestre\Tesi\myThesis\csv\measured_rotation.csv'
time, theta_deg = read_measurements(filename)

# Converti theta da gradi a radianti
theta = np.deg2rad(theta_deg)

# Stima iniziale dei parametri
initial_guess = compute_initial_guess(time, theta)

# Esegui il fit dei parametri usando curve_fit
popt, pcov = curve_fit(damped_oscillator_variable, time, theta, p0=initial_guess, maxfev=2000000)

# Estrai i parametri ottimali
A_opt, beta0_opt, beta1_opt, omega0_opt, omega1_opt, phi_opt, offset_opt = popt

# Calcola i valori del fit
fit_values_rad = damped_oscillator_variable(time, *popt)

# Salva i parametri in un file CSV
param_filename = r'C:\Users\giuse\OneDrive\Desktop\personale\Triennale\Terzo anno\Secondo semestre\Tesi\myThesis\csv\parameters.csv'
with open(param_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['A', A_opt])
    writer.writerow(['beta0', beta0_opt])
    writer.writerow(['beta1', beta1_opt])
    writer.writerow(['omega0', omega0_opt])
    writer.writerow(['omega1', omega1_opt])
    writer.writerow(['phi', phi_opt])
    writer.writerow(['offset', offset_opt])

# Leggi i parametri dal file CSV (opzionale)
params = {}
with open(param_filename, mode='r') as file:
    reader = csv.reader(file)
    for row in reader:
        params[row[0]] = float(row[1])

# Converti i valori del fit da radianti a gradi
fit_values_deg = np.rad2deg(fit_values_rad)

# Calcola i residui (differenza tra dati originali e fit)
residuals = theta_deg - fit_values_deg

# Calcola la somma dei residui al quadrato (SSR)
SSR = np.sum(residuals ** 2)

# Stampa la somma dei residui al quadrato
print(f'Somma dei residui al quadrato (SSR): {SSR:.5f}')

# Testo dei parametri per l'annotazione
param_text = (
    f'A = {A_opt:.2f} rad = {np.rad2deg(A_opt):.2f}$^\\circ$\n'
    f'$\\beta_0$ = {beta0_opt:.4f} s$^{{-1}}$\n'
    f'$\\beta_1$ = {beta1_opt:.6f} s$^{{-2}}$\n'
    f'$\\omega_0$ = {omega0_opt:.5f} rad/s = {np.rad2deg(omega0_opt):.2f}$^\\circ$/s\n'
    f'$\\omega_1$ = {omega1_opt:.5f} rad/s$^2$ = {np.rad2deg(omega1_opt):.2f}$^\\circ$/s$^2$\n'
    f'$\\phi$ = {phi_opt:.2f} rad = {np.rad2deg(phi_opt):.2f}$^\\circ$\n'
    f'$offset$ = {offset_opt:.2f} rad = {np.rad2deg(offset_opt):.2f}$^\\circ$'
)

# Plot dei dati originali e del fit
plt.figure(figsize=(12, 6))
plt.plot(time, theta_deg, label='Dati Originali', color='blue')
plt.plot(time, fit_values_deg, label='Fit', linestyle='--', color='orange')
plt.xlabel('Tempo (s)')
plt.ylabel('Theta (deg)')
plt.title('Confronto tra Dati Originali e Fit')
plt.legend()
plt.grid(True)

# Aggiungi l'annotazione dei parametri al grafico
plt.text(0.75, 0.87, param_text, transform=plt.gca().transAxes, fontsize=10,
         verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

plt.show()

# Plot dei residui con SSR nel titolo
plt.figure(figsize=(12, 6))
plt.plot(time, residuals, label='Residui', color='darkgreen')
plt.xlabel('Tempo (s)')
plt.ylabel('Residui (deg)')
plt.title(f'Residui tra Dati Originali e Fit (SSR: {SSR:.2f})')
plt.axhline(0, color='black', linestyle='--', linewidth=1)
plt.grid(True)
plt.legend()
plt.show()

# Calcolo di beta(t) e omega(t)
beta_t = beta0_opt + beta1_opt * time
omega_t = omega0_opt + omega1_opt * time

# Plot di beta(t)
plt.figure(figsize=(12, 6))
plt.plot(time, beta_t, label=r'$\beta(t)$')
plt.xlabel('Tempo (s)')
plt.ylabel(r'$\beta(t)$ (s$^{-1}$)')
plt.title('Andamento di $\beta(t)$ nel tempo')
plt.grid(True)
plt.legend()
plt.show()

# Plot di omega(t)
plt.figure(figsize=(12, 6))
plt.plot(time, omega_t, label=r'$\omega(t)$')
plt.xlabel('Tempo (s)')
plt.ylabel(r'$\omega(t)$ (rad/s)')
plt.title('Andamento di $\omega(t)$ nel tempo')
plt.grid(True)
plt.legend()
plt.show()











