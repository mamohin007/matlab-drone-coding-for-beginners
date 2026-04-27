clc;
clear;
close all;

% Parameters
m = 0.75;          % kg
g = 9.81;          % m/s^2
dt = 0.01;         % s
t_end = 8;         % s
t = 0:dt:t_end;

% Controller gains
Kp = 8;
Kd = 5;

% Reference altitude
z_ref = 1.5;       % m

% Preallocate arrays
z  = zeros(size(t));   % altitude
vz = zeros(size(t));   % vertical velocity
az = zeros(size(t));   % vertical acceleration
T  = zeros(size(t));   % total thrust
e  = zeros(size(t));   % altitude error

% Initial conditions
z(1) = 0;
vz(1) = 0;

% Simulation loop
for k = 1:length(t)-1
    
    % Altitude error
    e(k) = z_ref - z(k);
    
    % PD controller
    u = Kp*e(k) - Kd*vz(k);
    
    % Total thrust
    T(k) = m*g + u;
    
    % Thrust saturation
    T(k) = max(0, min(T(k), 20));
    
    % Vertical dynamics
    az(k) = (T(k) - m*g)/m;
    
    % Euler integration
    vz(k+1) = vz(k) + az(k)*dt;
    z(k+1) = z(k) + vz(k+1)*dt;
end

% Final values
e(end)  = z_ref - z(end);
T(end)  = T(end-1);
az(end) = az(end-1);

% Plot altitude
figure;
plot(t, z, 'LineWidth', 2);
hold on;
yline(z_ref, '--', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('PD altitude control response');
legend('z', 'z_{ref}');

% Plot thrust
figure;
plot(t, T, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Total thrust (N)');
title('PD control thrust history');

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

% Display final values
fprintf('Final altitude = %.4f m\n', z(end));
fprintf('Final altitude error = %.4f m\n', e(end));