function [state_mean_i_k,state_cov_i_k] = time_update(state_mean_i,state_cov_i,state_noise_cov,dt)

A = [1 0 dt 0 0 0;0 1 0 dt 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 dt;0 0 0 0 0 1];
state_mean_i_k = A*state_mean_i;
state_cov_i_k = A*state_cov_i*A.' + state_noise_cov;

end