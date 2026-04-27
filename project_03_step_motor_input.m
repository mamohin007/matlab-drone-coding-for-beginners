clc;
clear;
close all;

% Quadcopter parameters
m = 0.75;          % mass (kg)
g = 9.81;          % gravity (m/s^2)
kf = 1.8e-5;       % thrust coefficient

% Simulation settings
dt = 0.01;         % time step (s)
t = 0:dt:5;        % simulation time (s)

% State variables
z = zeros(size(t));      % altitude (m)
vz = zeros(size(t));     % vertical velocity (m/s)
az = zeros(size(t));     % vertical acceleration (m/s^2)

% Motor speed history
omega = zeros(size(t));  % motor speed (rad/s)

% Simulation loop
for k = 1:length(t)-1
    
    % Step motor-speed input
    if t(k) < 2
        omega(k) = 320;   % near hover
    else
        omega(k) = 380;   % climb
    end
    
    % Compute thrust
    T = 4 * kf * omega(k)^2;
    
    % Compute acceleration
    az(k) = (T - m*g) / m;
    
    % Euler integration
    vz(k+1) = vz(k) + az(k) * dt;
    z(k+1) = z(k) + vz(k+1) * dt;
end

% Fill final values for plotting
omega(end) = omega(end-1);
az(end) = az(end-1);

% Plot altitude
figure;
plot(t, z, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude response to motor speed change');

% Plot motor speed
figure;
plot(t, omega, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Motor speed (rad/s)');
title('Motor speed input');

% Plot acceleration
figure;
plot(t, az, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Vertical acceleration (m/s^2)');
title('Vertical acceleration response');

% Plot velocity
figure;
plot(t, vz, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Vertical velocity (m/s)');
title('Vertical velocity response');