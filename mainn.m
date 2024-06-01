close all;
clear all;
clc;


sensor_coordinates = [0 0];
stick_center_coordinates = [10 10];
body_velocity = [0.2 0.2];
body_acceleration = [0 0];
w_initial = pi/4;
angular_acceleration = 0;
phi_initial = pi/6;
stick_length = 5;
sampling_rate = 10;
measurement_time = 50;
error_variance_range = 0.1;
error_variance_phi = 0.01;
error_variance_velocity = 0.01;
radar_measurements_plots = "no";
radar_measurements_plots_pause = 0;


x_initial = 12;
y_initial = 8;
v_x_initial = 0.1;
v_y_initial = 0.1;
theta_initial = 0.1;
omega_initial = 0.1;
P_initial = diag([10 10 10 10 10 10]);
state_noise_cov = diag([1 1 1 1 0.1 0.1])./sampling_rate;
measurement_noise_cov = diag([0.2 0.02 0.02]);
measurement_noise_cov_wout_r_dot = diag([0.2 0.02]);
algorithm_measurement_plots = "no";
algorithm_measurement_plots_pause = 0;
iekf_iteration = 3;
monte_carlo_runs = 100;

x_initial = [x_initial;y_initial;v_x_initial;v_y_initial;theta_initial;omega_initial];
[measurements,ground_truths,ground_truths_states] = radar_sensor_measurements(sensor_coordinates,stick_center_coordinates,body_velocity,body_acceleration,w_initial,angular_acceleration,phi_initial,stick_length,sampling_rate,measurement_time,error_variance_range,error_variance_phi,error_variance_velocity,radar_measurements_plots,radar_measurements_plots_pause);

dt = 1/sampling_rate; 
t = 0:dt:measurement_time; 
state_means_ekf_mat = zeros(length(x_initial),length(t));
state_means_iekf_mat = zeros(length(x_initial),length(t));
state_means_ekf_wout_r_dot_mat = zeros(length(x_initial),length(t));
state_means_iekf_wout_r_dot_mat = zeros(length(x_initial),length(t));

parfor k = 1:monte_carlo_runs
[state_means_ekf,state_covs_ekf] = ekf(measurements,sensor_coordinates,measurement_time,sampling_rate,x_initial,P_initial,stick_length,state_noise_cov,measurement_noise_cov,algorithm_measurement_plots,algorithm_measurement_plots_pause,ground_truths_states);
[state_means_iekf,state_covs_iekf] = iekf(measurements,sensor_coordinates,measurement_time,sampling_rate,x_initial,P_initial,stick_length,state_noise_cov,measurement_noise_cov,iekf_iteration,algorithm_measurement_plots,algorithm_measurement_plots_pause,ground_truths_states);
[state_means_ekf_wout_r_dot,state_covs_ekf_wout_r_dot] = ekf__wout_r_dot(measurements,sensor_coordinates,measurement_time,sampling_rate,x_initial,P_initial,stick_length,state_noise_cov,measurement_noise_cov_wout_r_dot,algorithm_measurement_plots,algorithm_measurement_plots_pause,ground_truths_states);
[state_means_iekf_wout_r_dot,state_covs_iekf_wout_r_dot] = iekf_wout_r_dot(measurements,sensor_coordinates,measurement_time,sampling_rate,x_initial,P_initial,stick_length,state_noise_cov,measurement_noise_cov_wout_r_dot,iekf_iteration,algorithm_measurement_plots,algorithm_measurement_plots_pause,ground_truths_states);
state_means_ekf_mat = state_means_ekf_mat + state_means_ekf;
state_means_iekf_mat = state_means_iekf_mat + state_means_iekf;
state_means_ekf_wout_r_dot_mat = state_means_ekf_wout_r_dot_mat + state_means_ekf_wout_r_dot;
state_means_iekf_wout_r_dot_mat = state_means_iekf_wout_r_dot_mat + state_means_iekf_wout_r_dot;
end
state_means_ekf = state_means_ekf_mat./monte_carlo_runs;
state_means_iekf = state_means_iekf_mat./monte_carlo_runs;
state_means_ekf_wout_r_dot = state_means_ekf_wout_r_dot_mat./monte_carlo_runs;
state_means_iekf_wout_r_dot = state_means_iekf_wout_r_dot_mat./monte_carlo_runs;

plots(sampling_rate,measurement_time,ground_truths_states,state_means_ekf,state_means_iekf,state_means_ekf_wout_r_dot,state_means_iekf_wout_r_dot)
