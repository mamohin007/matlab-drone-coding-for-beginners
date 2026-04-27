clc;
clear;
close all;

% Quadcopter parameters
m = 0.75;          % mass in kg
g = 9.81;          % gravity in m/s^2
kf = 1.8e-5;       % thrust coefficient

% Motor speed range
omega = linspace(0, 800, 200);   % rad/s

% Total thrust from 4 equal motors
T_total = 4 * kf * omega.^2;

% Weight of the quadcopter
W = m * g;

% Find approximate hover speed
[~, idx] = min(abs(T_total - W));
omega_hover = omega(idx);

% Plot total thrust and weight
figure;
plot(omega, T_total, 'LineWidth', 2);
hold on;
yline(W, '--', 'LineWidth', 2);
grid on;

xlabel('Motor speed, \omega (rad/s)');
ylabel('Force (N)');
title('Quadcopter total thrust vs motor speed');
legend('Total thrust', 'Weight');

% Display hover speed
fprintf('Approximate hover speed = %.2f rad/s\n', omega_hover);

% Excess thrust
T_excess = T_total - W;

% Plot excess thrust
figure;
plot(omega, T_excess, 'LineWidth', 2);
grid on;

xlabel('Motor speed (rad/s)');
ylabel('Excess thrust (N)');
title('Excess thrust vs motor speed');