clc;
clear;
close all;

% Parameters
m = 0.75;
g = 9.81;
kf = 1.8e-5;

dt = 0.01;
t = 0:dt:5;

%% Case A: Step input
z_step = zeros(size(t));
vz_step = zeros(size(t));

for k = 1:length(t)-1
    
    % Step input
    if t(k) < 2
        omega = 320;   % rad/s
    else
        omega = 380;   % rad/s
    end
    
    T = 4 * kf * omega^2;
    az = (T - m*g) / m;
    
    vz_step(k+1) = vz_step(k) + az*dt;
    z_step(k+1) = z_step(k) + vz_step(k+1)*dt;
end

%% Case B: Ramp input
z_ramp = zeros(size(t));
vz_ramp = zeros(size(t));

for k = 1:length(t)-1
    
    % Ramp input
    if t(k) < 2
        omega = 320;                     % rad/s
    elseif t(k) < 4
        omega = 320 + 30*(t(k)-2);      % rad/s
    else
        omega = 380;                    % rad/s
    end
    
    T = 4 * kf * omega^2;
    az = (T - m*g) / m;
    
    vz_ramp(k+1) = vz_ramp(k) + az*dt;
    z_ramp(k+1) = z_ramp(k) + vz_ramp(k+1)*dt;
end

% Fill final values for clean printout
z_step(end) = z_step(end-1);
z_ramp(end) = z_ramp(end-1);

% Plot altitude comparison
figure;
plot(t, z_step, 'LineWidth', 2);
hold on;
plot(t, z_ramp, '--', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Step vs ramp input comparison');
legend('Step input', 'Ramp input');

% Plot velocity comparison
figure;
plot(t, vz_step, 'LineWidth', 2);
hold on;
plot(t, vz_ramp, '--', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Vertical velocity (m/s)');
title('Step vs ramp velocity comparison');
legend('Step input', 'Ramp input');

% Print final altitudes
fprintf('Final altitude (step) = %.4f m\n', z_step(end));
fprintf('Final altitude (ramp) = %.4f m\n', z_ramp(end));
