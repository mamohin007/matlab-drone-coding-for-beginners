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

% States
z = zeros(size(t));       % altitude (m)
vz = zeros(size(t));      % vertical velocity (m/s)
az = zeros(size(t));      % vertical acceleration (m/s^2)

% Input and thrust history
omega = zeros(size(t));   % motor speed (rad/s)
T_hist = zeros(size(t));  % thrust history (N)

% Simulation loop
for k = 1:length(t)-1
    
    % Motor speed profile
    if t(k) < 2
        omega(k) = 320;                   % constant region
    elseif t(k) < 4
        omega(k) = 320 + 30*(t(k) - 2);  % linear ramp
    else
        omega(k) = 380;                   % constant region
    end
    
    % Thrust
    T_hist(k) = 4 * kf * omega(k)^2;
    
    % Acceleration
    az(k) = (T_hist(k) - m*g) / m;
    
    % Euler integration
    vz(k+1) = vz(k) + az(k)*dt;
    z(k+1) = z(k) + vz(k+1)*dt;
end

% Fill final values
omega(end) = omega(end-1);
az(end) = az(end-1);
T_hist(end) = T_hist(end-1);

% Plot motor speed
figure;
plot(t, omega, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Motor speed (rad/s)');
title('Motor speed ramp input');

% Plot altitude
figure;
plot(t, z, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude response');

% Plot vertical velocity
figure;
plot(t, vz, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Vertical velocity (m/s)');
title('Vertical velocity response');

% Plot vertical acceleration
figure;
plot(t, az, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Vertical acceleration (m/s^2)');
title('Vertical acceleration response');

% Plot thrust
figure;
plot(t, T_hist, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Thrust (N)');
title('Thrust response');

% Print final values
fprintf('Final altitude = %.4f m\n', z(end));
fprintf('Final velocity = %.4f m/s\n', vz(end));
fprintf('Final thrust = %.4f N\n', T_hist(end));
fprintf('Final motor speed = %.4f rad/s\n', omega(end));