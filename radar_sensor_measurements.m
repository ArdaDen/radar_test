function [measurements,ground_truths,ground_truths_states] = radar_sensor_measurements(sensor_coor,stick_center_coor,body_vel,body_acc,w_initial,angular_acc,phi_initial,stick_length,sampling_rate,measurement_time,error_variance_range,error_variance_phi,error_variance_velocity,animation,radar_measurements_plots_pause)

%% Variables
dt = 1/sampling_rate; % Sampling period
t = 0:dt:measurement_time; % Measurement time interval
stick_t_x_c = stick_length/2*cos(phi_initial+(w_initial + angular_acc*t/2).*t); % Stick tip x coordinate when stick center taken as origin
stick_t_y_c = stick_length/2*sin(phi_initial+(w_initial + angular_acc*t/2).*t); % Stick tip y coordinate when stick center taken as origin
stick_tip_x = stick_center_coor(1) + stick_t_x_c + (body_vel(1) + body_acc(1)*t/2).*t; % Stick tip x coordinate
stick_tip_y = stick_center_coor(2) + stick_t_y_c + (body_vel(2) + body_acc(2)*t/2).*t; % Stick tip y coordinate
vel_vec = [(w_initial + angular_acc*t).*(stick_length./2).*cos(phi_initial+(w_initial + angular_acc*t/2).*t+ pi/2) + (body_vel(1) + body_acc(1)*t)...
    ;(w_initial + angular_acc*t).*(stick_length./2).*sin(phi_initial+(w_initial + angular_acc*t/2).*t+ pi/2) + (body_vel(2) + body_acc(2)*t)]; % Stick tip velocity vector 
sensor_vec = [stick_tip_x-sensor_coor(1);stick_tip_y-sensor_coor(2)]; % Vector from sensor to stick tip

%% Storing the cosine of the angles between velocity and sensor vectors in each time steps
cos_angle = zeros(1,length(t)); 
vel_norms = zeros(1,length(t));
for m = 1:length(t)
    cos_angle_m = dot(vel_vec(:,m),sensor_vec(:,m))/(norm(vel_vec(:,m))*norm(sensor_vec(:,m)));
    cos_angle(m) = cos_angle_m;
    vel_norms(m) = sqrt(vel_vec(1,m)^2 + vel_vec(2,m)^2);
end

%% Measurement data range, angle and velocity with errors and ground truths
measurements = [((stick_tip_x-sensor_coor(1)).^2 + (stick_tip_y-sensor_coor(2)).^2).^(1/2) + error_variance_range*randn(1,length(t));...% Range
    atan2((stick_tip_y-sensor_coor(2)),(stick_tip_x-sensor_coor(1))) + error_variance_phi*randn(1,length(t));... % Angle
    vel_norms.*cos_angle + error_variance_velocity*randn(1,length(t))]; % Velocity

ground_truths = [((stick_tip_x-sensor_coor(1)).^2 + (stick_tip_y-sensor_coor(2)).^2).^(1/2);...% Range
    atan2((stick_tip_y-sensor_coor(2)),(stick_tip_x-sensor_coor(1)));... % Angle
    vel_norms.*cos_angle]; % Velocity

ground_truths_states = [stick_center_coor(1) + (body_vel(1) + body_acc(1)*t/2).*t;... % P_x
    stick_center_coor(2) + (body_vel(2) + body_acc(2)*t/2).*t;... % P_y
    body_vel(1) + body_acc(1)*t;... % v_x
    body_vel(2) + body_acc(2)*t;... % v_y
    phi_initial+(w_initial + angular_acc*t/2).*t;... % theta
    w_initial + angular_acc*t]; % omega

if animation == "yes"
    %% Plots
    figure;
    for i = 1:length(t)
        x = measurements(1,i)*cos(measurements(2,i));
        y = measurements(1,i)*sin(measurements(2,i));
        x_true = ground_truths(1,i)*cos(ground_truths(2,i));
        y_true = ground_truths(1,i)*sin(ground_truths(2,i));
        pos_vector = [x-sensor_coor(1),y-sensor_coor(2)];
        pos_vector_true = [x_true-sensor_coor(1),y_true-sensor_coor(2)];
        subplot(2,1,1);
        plot(x,y,"Marker","*","Color","k");
        hold on;
        plot(sensor_coor(1),sensor_coor(2),"Marker","o","Color","b");
        plot([x x + 0.1*pos_vector/norm(pos_vector)*measurements(3,i)], [y y + 0.1*pos_vector/norm(pos_vector)*measurements(3,i)],"k");
        plot(stick_center_coor(1) + (body_vel(1) + body_acc(1)*t(i))*t(i),stick_center_coor(2) + (body_vel(2) + body_acc(2)*t(i))*t(i),"Marker","+","Color","r")
        title("Measurements with errors")
        xlabel("Metre(m)")
        ylabel("Metre(m)")
        subplot(2,1,2);
        plot(x_true,y_true,"Marker","*","Color","k");
        hold on;
        plot(sensor_coor(1),sensor_coor(2),"Marker","o","Color","b");
        plot([x_true x_true + 0.1*pos_vector_true/norm(pos_vector_true)*ground_truths(3,i)], [y_true y_true + 0.1*pos_vector_true/norm(pos_vector_true)*ground_truths(3,i)],"k");
        plot(stick_center_coor(1) + (body_vel(1) + body_acc(1)*t(i))*t(i),stick_center_coor(2) + (body_vel(2) + body_acc(2)*t(i))*t(i),"Marker","+","Color","r")
        title("Ground truths")
        xlabel("Metre(m)")
        ylabel("Metre(m)")
        pause(radar_measurements_plots_pause)
    end
    hold off;
end

end