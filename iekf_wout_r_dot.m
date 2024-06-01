%% Iterated Extended Kalman Filter Without Radial Velocity
function [state_means,state_covs] = iekf_wout_r_dot(measurements,sensor_coor,Time,sampling_rate,state_mean_i, ...
     state_cov_i,stick_length,state_noise_cov,measurement_noise_cov,iteration_number,animation,iekf_plots_pause,ground_truth_states)

%% Variables
measurements = measurements(1:2,:);
dt = 1/sampling_rate;
t = 0:dt:Time;
state_means = zeros(length(state_mean_i),length(t));
state_covs = zeros(length(state_mean_i),length(state_mean_i),length(t));

%% IEKF Algorithm
for i = 1:length(t)
    [state_mean_i_k,state_cov_i_k] = time_update(state_mean_i,state_cov_i,state_noise_cov,dt);
    for m = 1:iteration_number
    [state_mean_i_k,state_cov_i_k] = measurement_update_wout_r_dot(state_mean_i_k,state_cov_i_k,measurements(:,i),measurement_noise_cov,sensor_coor,stick_length);
    end
    state_means(:,i) = state_mean_i_k;
    state_covs(:,:,i) = state_cov_i_k;
    state_mean_i = state_mean_i_k;
    state_cov_i = state_cov_i_k;
end

%% Plots
if animation == "yes"
    figure;
    for i = 1:length(t)
        x = state_means(1,i);
        y = state_means(2,i);
        plot(x,y,"Marker","*");
        hold on;
        plot(sensor_coor(1),sensor_coor(2),"Marker","o");
        title("Predicted States")
        xlabel("Metre(m)")
        ylabel("Metre(m)")
        pause(iekf_plots_pause)
    end
    plot(ground_truth_states(1,:),ground_truth_states(2,:));
    hold off;
end
