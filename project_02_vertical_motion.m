clc;
clear;
close all;

% Quadcopter parameters
m = 0.75;          % mass (kg)
g = 9.81;          % gravity (m/s^2)
kf = 1.8e-5;       % thrust coefficient

% Motor speed (constant)
omega = 380;       % rad/s

% Compute total thrust
T = 4 * kf * omega^2;

% Simulation settings
dt = 0.01;         % time step (s)
t = 0:dt:5;        % simulation time (s)

% State variables
z = zeros(size(t));    % altitude (m)
vz = zeros(size(t));   % vertical velocity (m/s)
az = zeros(size(t));   % vertical acceleration (m/s^2)

% Simulation loop
for k = 1:length(t)-1
    
    % Vertical acceleration
    az_const = (T - m*g) / m;
    az(k) = az_const;
    
    % Euler integration
    vz(k+1) = vz(k) + az(k) * dt;
    z(k+1) = z(k) + vz(k+1) * dt;
end

% Fill final acceleration value for plotting
az(end) = az(end-1);

% Plot altitude
figure;
plot(t, z, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Quadcopter vertical motion');

% Plot vertical velocity
figure;
plot(t, vz, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Vertical velocity (m/s)');
title('Quadcopter vertical velocity');

% Plot vertical acceleration
figure;
plot(t, az, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Vertical acceleration (m/s^2)');
title('Quadcopter vertical acceleration');