- aruco_library: defines functions to detect and determine the pose and orientation of ArUco Markers using the OpenCV library.

- maglev_rotationCV: takes a video as input and uses the functions defined in aruco_library to identify the IDs of ArUco Markers, detect their pose and orientation, and generate an output video with the marked information. This script is named maglev_rotationCV because it was applied to a video featuring a levitating magnet, but it can be adapted to any video containing ArUco markers.

- ideal_harmonic_oscillator: generates the rotational response of a harmonic oscillator based on physical parameters provided by the user.

- model_estimation_naive: derives the optimal parameters of a harmonic oscillator from a continuous dataset containing pairs of (time, rotation angle). It is suitable only for ideal datasets.

- model_estimation_naive_4real_data: this is the model_estimation_naive script applied to a real dataset (demonstrating the inadequacy of the naive estimation for real data).

- model_estimation_simple: an improved version of model_estimation_naive, which also works for real datasets, as long as the noise is minimal and there are no nonlinear effects in the system.

- model_estimation_advanced: doesn't actually perform an advanced estimation but considers the optimal parameters of models with time-varying oscillator parameters. This generally improves parameter estimation for systems with disturbances and/or nonlinear effects.

- Kalman: this script receives real measurements and the estimated parameters of the corresponding harmonic oscillator model. Using this information, it tunes the measurement and process noise, performs Kalman filtering, and provides plots and numerical information to evaluate the quality of the filtering.

- measured_rotation.csv: an example of a real dataset generated with maglev_rotationCV from a video.

- parameters_constant.csv: parameters estimated from measured_rotation.csv using model_estimation_simple.py.

- parameters_variable.csv: parameters estimated from measured_rotation.csv using model_estimation_advanced.py.

 
