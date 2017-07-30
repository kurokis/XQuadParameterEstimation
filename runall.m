clear all;

run('./settings.m'); % load settings

if use_simulator % for debug purpose
    run('./src/simulator.m'); % simulate motor response and write output to csv
    output_filename = 'simulator.csv';
else % read log data
    run('./src/process_log.m'); % convert sensor readings to designated format and write output to csv
    output_filename = strrep(log_filename,'.csv',processed_log_suffix);
end

run('./src/parameter_estimation.m'); % perform parameter estimation