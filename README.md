X-Quad Parameter Estimation
--

This program estimates the actuation matrix of a quadcopter in X configuration using propeller commands and IMU data.
Additionally, mass, moment of inertia and motor time constant can be estimated, given following parameters are measured beforehand.
 - l: lever arm length of motors (m)
 - KT: coefficient of proportionality for scaling delta_omega (rad/s) to thrust (N), where delta_omega is change in rotor speed from hovering condition
 - KQ: coefficient of proportionality for scaling delta_omega (rad/s) to reaction torque (N), where delta_omega is change in rotor speed from hovering condition

Required sensor data
--

In order to perform parameter estimation, a flight log needs to be obtained in the following CSV format.

column 1: don't care  
column 2: timestamp in MICROSECONDS  
column 3: motor setpoint for rotor 1 (rad/s)  
column 4: motor setpoint for rotor 2 (rad/s)  
column 5: motor setpoint for rotor 3 (rad/s)  
column 6: motor setpoint for rotor 4 (rad/s)  
column 7: accelerometer x-axis readings (g)  
column 8: accelerometer y-axis readings (g)  
column 9: accelerometer z-axis readings (g)  
column 10: gyro x-axis readings (rad/s)  
column 11: gyro x-axis readings (rad/s)  
column 12: gyro x-axis readings (rad/s)  

How to use
--

1. Put log file to appropriate directory

   Copy your log file to the same folder as where 'settings.m' is located in.
1. Change settings

   Motor setpoint for hovering condition needs to be estimated manually. From recorded log file, estimate motor setpoint for hover and record it to 'setpoint_at_hover' in 'settings.m'.

   Also, change 'log_filename' to the name of the log file and make sure 'use_simulator' is set to 'false'.
1. Run

   Start Matlab, move to the right directory, and type 'runall'.
