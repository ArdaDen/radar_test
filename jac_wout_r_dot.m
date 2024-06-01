function H = jac_wout_r_dot(vec,sensor_pos,stick_length)
a = vec(1);
b = vec(2);
c = vec(3);
d = vec(4);
e = vec(5);
f = vec(6);
x = sensor_pos(1);
y = sensor_pos(2);
s = stick_length;

H = [[(2*real(a) - 2*real(x) + cos(real(e))*real(s))/(2*((real(a) - real(x) + (cos(real(e))*real(s))/2)^2 + (real(b) - real(y) + (sin(real(e))*real(s))/2)^2)^(1/2)), (2*real(b) - 2*real(y) + sin(real(e))*real(s))/(2*((real(a) - real(x) + (cos(real(e))*real(s))/2)^2 + (real(b) - real(y) + (sin(real(e))*real(s))/2)^2)^(1/2)), 0, 0,                                                                                                      -(sin(real(e))*real(s)*(real(a) - real(x) + (cos(real(e))*real(s))/2) - cos(real(e))*real(s)*(real(b) - real(y) + (sin(real(e))*real(s))/2))/(2*((real(a) - real(x) + (cos(real(e))*real(s))/2)^2 + (real(b) - real(y) + (sin(real(e))*real(s))/2)^2)^(1/2)), 0];...
[         -(real(b) - real(y) + (sin(real(e))*real(s))/2)/((real(a) - real(x) + (cos(real(e))*real(s))/2)^2 + (real(b) - real(y) + (sin(real(e))*real(s))/2)^2),           (real(a) - real(x) + (cos(real(e))*real(s))/2)/((real(a) - real(x) + (cos(real(e))*real(s))/2)^2 + (real(b) - real(y) + (sin(real(e))*real(s))/2)^2), 0, 0, (((cos(real(e))*real(s))/(2*(real(a) - real(x) + (cos(real(e))*real(s))/2)) + (sin(real(e))*real(s)*(real(b) - real(y) + (sin(real(e))*real(s))/2))/(2*(real(a) - real(x) + (cos(real(e))*real(s))/2)^2))*(real(a) - real(x) + (cos(real(e))*real(s))/2)^2)/((real(a) - real(x) + (cos(real(e))*real(s))/2)^2 + (real(b) - real(y) + (sin(real(e))*real(s))/2)^2), 0]];
end