function [state_mean_i,state_cov_i] = measurement_update_wout_r_dot(state_mean_i_k,state_cov_i_k,measurements,measurement_noise_cov,sensor_coor,stick_length)
h = [sqrt((state_mean_i_k(1) + stick_length/2*cos(state_mean_i_k(5)) - sensor_coor(1))^2 + (state_mean_i_k(2) + stick_length/2*sin(state_mean_i_k(5)) - sensor_coor(2))^2);...
    atan2(state_mean_i_k(2) + stick_length/2*sin(state_mean_i_k(5)) - sensor_coor(2),state_mean_i_k(1) + stick_length/2*cos(state_mean_i_k(5)) - sensor_coor(1))];
y_k_in = measurements-h;
H = jac_wout_r_dot(state_mean_i_k,sensor_coor,stick_length);
S_k = H*state_cov_i_k*H.' + measurement_noise_cov;
K_k = state_cov_i_k*H.'/S_k;
state_mean_i = state_mean_i_k + K_k*y_k_in;
state_cov_i = (eye(length(K_k(:,1)))-K_k*H)*state_cov_i_k;
end