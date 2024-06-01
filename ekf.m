%% Extended Kalman Filter
function [state_means,state_covs] = ekf(measurements,sensor_coor,Time,sampling_rate,state_mean_i, ...
     state_cov_i,stick_length,state_noise_cov,measurement_noise_cov,animation,ekf_plots_pause,ground_truth_states)

%% Variables
dt = 1/sampling_rate;
t = 0:dt:Time;
state_means = zeros(length(state_mean_i),length(t));
state_covs = zeros(length(state_mean_i),length(state_mean_i),length(t));

%% EKF Algorithm
for i = 1:length(t)
    [state_mean_i_k,state_cov_i_k] = time_update(state_mean_i,state_cov_i,state_noise_cov,dt);
    [state_mean_i,state_cov_i] = measurement_update(state_mean_i_k,state_cov_i_k,measurements(:,i),measurement_noise_cov,sensor_coor,stick_length);
    state_means(:,i) = state_mean_i;
    state_covs(:,:,i) = state_cov_i;
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
        pause(ekf_plots_pause)
    end
    plot(ground_truth_states(1,:),ground_truth_states(2,:));
    hold off;
end

end


