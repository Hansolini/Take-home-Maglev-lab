This folder contains code related to the off-chip implementation described under "Implementation study: off-chip computations" in Chapter 15 - Controller implementation.

"Transmission_Speed_Test" includes the code for testing general data transmission time between Teensy and MATLAB. Start the Teensy code first, and then the MATLAB
script shortly thereafter. Follow method outlined in thesis.

"Data_sending_test_final" includes the code used for testing the transmission time when meaningful data was actually transmitted, that is the
test outlined where Teensy generates 10 random states and MATLAB calculates input based on them and send it back to the Teensy.
The code is divided into Arduino code for the Teensy and MATLAB code for running on a computer.
To conduct a test, start running the Teensy code "Data_sending_test_final.ino", and wait for a couple of seconds before running the MATLAB script.

The results should arrive within a couple of seconds. The MATLAB script is also equipped with plotting of some of the results. The plot where
this implementation approach is compared to the on-chip approach also includes data from .csv-files. This data is collected from the on-chip
implementation direction.

-Pål Ivar Delphin Kværnø Fosmo, May 27th 2024