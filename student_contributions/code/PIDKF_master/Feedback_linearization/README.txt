This folder contains code related to the Feedback linearization controller described under "Feedback linearization" in Chapter 14 - Stabilizing the system.
The code in this folder constitutes the simulator environment in which the feedback linearization controller was developed, and the related
stability analysis was done.

The main simulator is a Simulink-file called "fl.slx". In order to run the simulator, this folder must be added to the MATLAB path (with subfolders), and
"pole_placement.m" must be run. "pole_placement.m" includes all code related to placement of the poles and other tuning of the controller.
"Dataset.mat" includes all variables needed for successful tuning and running of the simulator. This data is gathered from the live script "main.mlx" which
is under the "simulation"-folder in this repository.

"convergence_plot_func.m" contains the code developed for the method described in the "Stability analysis through simulation"-section.

"simulink_lqr.slx" contains a block diagram version of the LQR controller included in the main simulator code under "simulation" in this repository.

-Pål Ivar Delphin Kværnø Fosmo, May 27th 2024.