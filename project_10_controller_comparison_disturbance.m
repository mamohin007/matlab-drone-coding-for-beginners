clc;
clear;
close all;

% Parameters
m = 0.75;
g = 9.81;
dt = 0.01;
t_end = 8;
t = 0:dt:t_end;

% Reference
z_ref = 1.5;

% Base gains
Kp = 8;
Ki = 0.15;
Kd = 5;

% ---- P controller ----
Ki_P = 0;
Kd_P = 0;
z_P = run_altitude_case(t, dt, m, g, z_ref, Kp, Ki_P, Kd_P);

% ---- PD controller ----
Ki_PD = 0;
Kd_PD = Kd;
z_PD = run_altitude_case(t, dt, m, g, z_ref, Kp, Ki_PD, Kd_PD);

% ---- PI controller ----
Ki_PI = Ki;
Kd_PI = 0;
z_PI = run_altitude_case(t, dt, m, g, z_ref, Kp, Ki_PI, Kd_PI);

% ---- PID controller ----
Ki_PID = Ki;
Kd_PID = Kd;
z_PID = run_altitude_case(t, dt, m, g, z_ref, Kp, Ki_PID, Kd_PID);

% Plot altitude comparison
figure;
plot(t, z_P, 'LineWidth', 2);
hold on;
plot(t, z_PD, 'LineWidth', 2);
plot(t, z_PI, 'LineWidth', 2);
plot(t, z_PID, 'LineWidth', 2);
yline(z_ref, '--', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Controller comparison under disturbance');
legend('P', 'PD', 'PI', 'PID', 'z_{ref}');

% Error comparison
e_P = z_ref - z_P;
e_PD = z_ref - z_PD;
e_PI = z_ref - z_PI;
e_PID = z_ref - z_PID;

figure;
plot(t, e_P, 'LineWidth', 2);
hold on;
plot(t, e_PD, 'LineWidth', 2);
plot(t, e_PI, 'LineWidth', 2);
plot(t, e_PID, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Error (m)');
title('Controller error comparison under disturbance');
legend('P', 'PD', 'PI', 'PID');

% -------- Local function --------
function z = run_altitude_case(t, dt, m, g, z_ref, Kp, Ki, Kd)

    z = zeros(size(t));
    vz = zeros(size(t));
    az = zeros(size(t));
    T = zeros(size(t));
    e = zeros(size(t));
    e_int = zeros(size(t));

    z(1) = 0;
    vz(1) = 0;
    e_int(1) = 0;

    for k = 1:length(t)-1
        
        % Disturbance step
        if t(k) >= 2
            dist = -0.8;
        else
            dist = 0;
        end
        
        % Error
        e(k) = z_ref - z(k);
        
        % Integral update
        e_int(k+1) = e_int(k) + e(k)*dt;
        
        % Controller
        u = Kp*e(k) + Ki*e_int(k) - Kd*vz(k);
        
        % Thrust
        T(k) = m*g + u;
        T(k) = max(0, min(T(k), 20));
        
        % Dynamics
        az(k) = (T(k) - m*g + dist)/m;
        
        % Euler integration
        vz(k+1) = vz(k) + az(k)*dt;
        z(k+1) = z(k) + vz(k+1)*dt;
    end
end