
% --- Parameters to be changed for each analysis ---
log_filename = 'log.csv';
setpoint_at_hover = 500; % motor setpoint of rotor at hover
kt = 0.02; % this needs to be measured beforehand
kq = 0.0006; % this needs to be measured beforehand
l = 0.3; % this needs to be measured beforehand

use_simulator = true;
show_plot_process_log = false;
show_plot_model_estimation = true;



% --- Constants ---
% -- process log --
id_from_flightctrl = 1;
col_id = 1;
col_timestamp = 2;
col_setpoint_1 = 3; % motor setpoint of rotor 1 in rad/s
col_setpoint_2 = 4; % motor setpoint of rotor 2 in rad/s
col_setpoint_3 = 5; % motor setpoint of rotor 3 in rad/s
col_setpoint_4 = 6; % motor setpoint of rotor 4 in rad/s
col_accelerometer_x = 7; % body x-axis accelerometer readings in g (steady: 0)
col_accelerometer_y = 8; % body y-axis accelerometer readings in g (steady: 0)
col_accelerometer_z = 9; % body z-axis accelerometer readings in g (steady: -1)
col_gyro_x = 10; % body x-axis gyro readings in rad/s
col_gyro_y = 11; % body y-axis gyro readings in rad/s
col_gyro_z = 12; % body z-axis gyro readings in rad/s
processed_log_suffix = '_out.csv'; % string to replace '.csv'

% -- attitude kalman filter --
sigma_gyro = 0.005; % standard deviation of gyro noise in rad/s
sigma_accelerometer = 0.02; % standard deviation of accelerometer noise in rad/s

% -- model estimation --
Ix0 = 0.06; % assume Ix = Iy
Iz0 = 0.12;
m0 = 2.1;
tau0 = 0.1;
niter = 50; % number of iteration for EKF run using same data (higher the better, stop if parameters are converged)
