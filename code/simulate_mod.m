% Define curvature (k), rotation angle in the xy plane (phi), and arc length (s)
k = 1.1;       % Adjust this value as needed
phi = 3*pi/2;  % Adjust this value as needed
s = 1.3;       % Adjust this value as needed

% Calculate the radius of curvature (R)
R = 1 / k;

% Calculate the arc angle (theta)
theta = s / R;

% Define the starting point (x0, y0, z0)
x0 = 0;
y0 = 0 - R;
z0 = 0;

% Generate values of t from 0 to theta
t_values = linspace(0, theta, 100);

% Calculate the corresponding (x, y, z) points along the 3D arc
y_values = y0 + R * cos(t_values);
x_values = 0 * cos(t_values);
z_values = z0 + R * sin(t_values);
x =x0;
phi_x = [];
phi_y = [];
for i = 1:100
    phi_x(i) = cos(phi)*x + sin(phi) * y_values(i);
    phi_y(i) = sin(phi)*x_values(i) + cos(phi)* y_values(i);

end
% 
% mot_vals = 0 * cos(t_values);
% motorx = [];
% motory = [];
% for i = 1:25
%     motorx(i) = phi_x(end) + cos(phi)*x + sin(phi) * y_values(i);
%     motory(i) = sin(phi)*x_values(i) + cos(phi)* y_values(i);
% end

% Create a 3D plot
x_joint = linspace(phi_x(end), phi_y(end));  
y_joint = linspace(phi_y(end), phi_y(end)); 
z_joint = linspace(z_values(end), z_values(end) + 0.06, 25);

figure;
plot3(phi_x, phi_y, z_values);
% plok3(x_joint, y_joint, z_joint);
xlim([-2 2])
ylim([-2 2])
zlim([0 2])
xlabel('X-axis')
ylabel('Y-axis')
zlabel('Z-axis')
title('3D Plot of an Arc');
grid on;

% first_coord = [x_values(1), y_values(1), z_values(1)]