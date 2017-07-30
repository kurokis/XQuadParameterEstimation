M = csvread(strcat('../',log_filename));
timestamp = M(:,col_timestamp);

% compute sample time
[n,~] = size(timestamp);
dts = zeros(n-1,1);
for i = 2:n
    dts(i,:) = (timestamp(i)-timestamp(i-1))/1000000;
end
dt = mean(dts); clear dts;

% delta rotor speed command
delta_omega_cmd = zeros(n,4);
delta_omega_cmd(:,1) = M(:,col_setpoint_1) - setpoint_at_hover;
delta_omega_cmd(:,2) = M(:,col_setpoint_2) - setpoint_at_hover;
delta_omega_cmd(:,3) = M(:,col_setpoint_3) - setpoint_at_hover;
delta_omega_cmd(:,4) = M(:,col_setpoint_4) - setpoint_at_hover;

% compute wdot
accelerometer = [M(:,col_accelerometer_x) M(:,col_accelerometer_y) M(:,col_accelerometer_z)];
gyro = [M(:,col_gyro_x) M(:,col_gyro_y) M(:,col_gyro_z)];
run('attitude_kf.m'); % compute quaternion and world_acceleration
wdot_raw = world_acceleration_bodyframe(:,3); % unfiltered body z-axis acceleration
wdot = sgolayfilt(wdot_raw,5,41); % Savitzky-Golay filter

% compute pdot, qdot, rdot
gyro_filt = sgolayfilt(gyro,5,41); % Savitzky-Golay filter
pdot = zeros(n,1); qdot = zeros(n,1); rdot = zeros(n,1);
for i = 2:n
    pdot(i) = (gyro_filt(i,1) - gyro_filt(i-1,1))/dt;
    qdot(i) = (gyro_filt(i,2) - gyro_filt(i-1,2))/dt;
    rdot(i) = (gyro_filt(i,3) - gyro_filt(i-1,3))/dt;
end

% save processed log
processed_log_filename = strrep(strcat('../output/',log_filename),'.csv',processed_log_suffix);
csvwrite(processed_log_filename, [timestamp delta_omega_cmd wdot pdot qdot rdot]);

% for debug
if show_plot_process_log
    subplot(2,2,1); % wdot
    hold on;
    plot(wdot_raw);
    plot(wdot);
    grid;
    title('wdot');
    legend('wdot raw','wdot');
    
    subplot(2,2,2); % pdot
    hold on;
    plot(gyro(:,1));
    plot(gyro_filt(:,1));
    plot(pdot);
    grid;
    title('pdot')
    legend('gyro raw','gyro filtered','pdot');
    
    subplot(2,2,3); % qdot
    hold on;
    plot(gyro(:,2));
    plot(gyro_filt(:,2));
    plot(qdot);
    grid;
    title('qdot')
    legend('gyro raw','gyro filtered','qdot');
    
    subplot(2,2,4); % rdot
    hold on;
    plot(gyro(:,3));
    plot(gyro_filt(:,3));
    plot(rdot);
    grid;
    title('rdot')
    legend('gyro raw','gyro filtered','rdot');
end
