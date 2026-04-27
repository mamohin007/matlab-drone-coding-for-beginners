clc;
clear;
close all;

% Parameters
m = 0.75;          % kg
g = 9.81;          % m/s^2
dt = 0.01;         % s
t_end = 8;
t = 0:dt:t_end;

% Controller gains
Kp = 8;
Ki = 0.15;
Kd = 5;

% Reference altitude
z_ref = 1.5;       % m

% Constant disturbance force (downward)
dist = -0.8;       % N

% Preallocation arrays
z = zeros(size(t));
vz = zeros(size(t));
az = zeros(size(t));
T = zeros(size(t));
e = zeros(size(t));
e_int = zeros(size(t));

% Initial conditions
z(1) = 0;
vz(1) = 0;
e_int(1) = 0;

% Simulation loop
for k = 1:length(t)-1
    
    % Error
    e(k) = z_ref - z(k);
    
    % Integral update
    e_int(k+1) = e_int(k) + e(k)*dt;
    
    % PID controller
    u = Kp*e(k) + Ki*e_int(k) - Kd*vz(k);
    
    % Total thrust
    T(k) = m*g + u;
    
    % Thrust saturation
    T(k) = max(0, min(T(k), 20));
    
    % Vertical dynamics with disturbance
    az(k) = (T(k) - m*g + dist)/m;
    
    % Euler integration
    vz(k+1) = vz(k) + az(k)*dt;
    z(k+1) = z(k) + vz(k+1)*dt;
end

% Final values
e(end) = z_ref - z(end);
T(end) = T(end-1);
az(end) = az(end-1);

% Plot altitude
figure;
plot(t, z, 'LineWidth', 2);
hold on;
yline(z_ref, '--', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('PID altitude control response');
legend('z', 'z_{ref}');

% Plot thrust
figure;
plot(t, T, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Total thrust (N)');
title('PID control thrust history');

% Plot integral error
figure;
plot(t, e_int, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Integral error');
title('Integral of altitude error');

% Plot velocity
figure;
plot(t, vz, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Vertical velocity (m/s)');
title('Vertical velocity response');

% Plot acceleration
figure;
plot(t, az, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Vertical acceleration (m/s^2)');
title('Vertical acceleration response');

% Display final values
fprintf('Final altitude = %.4f m\n', z(end));
fprintf('Final altitude error = %.4f m\n', e(end));