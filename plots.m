function plots(sampling_rate,measurement_time,ground_truths_states,state_means_ekf,state_means_iekf,state_means_ekf_wout_r_dot,state_means_iekf_wout_r_dot)
dt = 1/sampling_rate; 
t = 0:dt:measurement_time; 
headings = ["X Coordinates","Y Coordinates","X Velocity","Y Velocity","Angle","Angular Velocity"];
ylabels = ["Metre(m)","Metre(m)","Velocity(m/s)","Velocity(m/s)","Angle(rad)","Angular Velocity(rad/s)"];
figure;
tiledlayout(3,2);
for i = 1:length(state_means_ekf(:,1))
ax = nexttile;
plot(t,ground_truths_states(i,:));
hold on;
plot(t,state_means_ekf(i,:));
plot(t,state_means_iekf(i,:));
plot(t,state_means_ekf_wout_r_dot(i,:));
plot(t,state_means_iekf_wout_r_dot(i,:));
title(headings(i))
xlabel("Time(s)")
ylabel(ylabels(i))
legend("Ground Truth","EKF","IEKF","EKF Without Radial Velocity","IEKF Without Radial Velocity")
end

error_mat = zeros(length(ground_truths_states(:,1)),4);
figure;
tiledlayout(3,2);
for i = 1:length(state_means_ekf(:,1))
ax = nexttile;
plot(t,abs(ground_truths_states(i,:)-state_means_ekf(i,:)));
ekf_error = sum(abs(ground_truths_states(i,:)-state_means_ekf(i,:)))/length(t);
hold on;
plot(t,abs(ground_truths_states(i,:)-state_means_iekf(i,:)));
iekf_error = sum(abs(ground_truths_states(i,:)-state_means_iekf(i,:)))/length(t);
plot(t,abs(ground_truths_states(i,:)-state_means_ekf_wout_r_dot(i,:)));
ekf_error_r = sum(abs(ground_truths_states(i,:)-state_means_ekf_wout_r_dot(i,:)))/length(t);
plot(t,abs(ground_truths_states(i,:)-state_means_iekf_wout_r_dot(i,:)));
iekf_error_r = sum(abs(ground_truths_states(i,:)-state_means_iekf_wout_r_dot(i,:)))/length(t);
mat = [ekf_error,iekf_error,ekf_error_r,iekf_error_r];
error_mat(i,:) = mat;
title("Errors In " + headings(i))
xlabel("Time(s)")
ylabel(ylabels(i))
legend("EKF","IEKF","EKF Without Radial Velocity","IEKF Without Radial Velocity")
end

sum_ekf = zeros(1,length(t));
sum_iekf = zeros(1,length(t));
sum_ekf_wout_r_dot = zeros(1,length(t));
sum_iekf_wout_r_dot = zeros(1,length(t));
for i = 1:length(t)
    sum_ekf(i) = norm(abs(ground_truths_states(:,i)-state_means_ekf(:,i)));
    sum_iekf(i) = norm(abs(ground_truths_states(:,i)-state_means_iekf(:,i)));
    sum_ekf_wout_r_dot(i) = norm(abs(ground_truths_states(:,i)-state_means_ekf_wout_r_dot(:,i)));
    sum_iekf_wout_r_dot(i) = norm(abs(ground_truths_states(:,i)-state_means_iekf_wout_r_dot(:,i)));
end
figure
plot(t,sum_ekf);
hold on;
plot(t,sum_iekf);
plot(t,sum_ekf_wout_r_dot);
plot(t,sum_iekf_wout_r_dot);
title("Total State Errors")
xlabel("Time(s)")
ylabel("Error Norms")
legend("EKF","IEKF","EKF Without Radial Velocity","IEKF Without Radial Velocity")

Algorithms = ["EKF";"IEKF";"EKF Without Radial Velocity";"IEKF Without Radial Velocity"];
X_Coordinates = error_mat(1,:)';
Y_Coordinates = error_mat(2,:)';
X_Velocity = error_mat(3,:)';
Y_Velocity = error_mat(4,:)';
Angle = error_mat(5,:)';
Angular_Velocity = error_mat(6,:)';
T = table(Algorithms,X_Coordinates,Y_Coordinates,X_Velocity,Y_Velocity,Angle,Angular_Velocity)

end
