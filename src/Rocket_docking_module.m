clc;
clear;
close all;

% Constants
g = 9.81;       % Gravity (m/s^2)
m = 1600000;     % Rocket mass (kg)
R = 35;         % Rocket radius (m)
I = (m * (2 * R)^2) / 12;  % Moment of inertia

% Define the state-space matrices
A = [0 1 0 0 0 0; 
     0 0 0 0 g 0; 
     0 0 0 1 0 0; 
     0 0 0 0 0 0; 
     0 0 0 0 0 1; 
     0 0 0 0 0 0];

B = [0 0; 
     0 g; 
     0 0; 
     1/m 0; 
     0 0; 
     0 m*g*R/I];

C = eye(6);   % Output matrix (we're interested in all states)
D = zeros(6, 2);  % No direct feedthrough



% Initial conditions [x, x', y, y', phi, phi']
x0 = [1000; 10; 1000; 20; 0; 0];

% LQR design
Q = diag([100000, 1000000, 1000, 100000, 100000000000, 100000000000]);  % Penalizing system
R = diag([1e-12, 1]);  % Penalizing control effort

K = lqr(A, B, Q, R);  % LQR controller



% Define the system
sys = ss((A - B * K), B, C, D);

t = 0:0.3:400;
[y, t, x] = initial(sys, x0, t);



% Create the figure for animation
figure;
% axis([-2 15 -2 15]);  % Set the axis limits for a better view
hold on;
grid on;

x_pos = x(:, 1);
y_pos = y(:, 3);
phi_pos = x(:, 5);

% Plot the rocket's path
path_plot = plot(NaN, NaN, 'r-', 'LineWidth', 2); % Rocket path (initialize empty plot)
rocket_plot = plot(x_pos(1), y_pos(1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');  % Rocket marker
rocket_plot_up = plot(x_pos(1), y_pos(1), 'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g');

% Add labels and title
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Rocket Trajectory with Trace');
legend('Rocket Path', 'Rocket Position');
axis equal;

% Simulate movement step by step with trace
for i = 1:length(t)
    % Update the rocket marker position
    rocket_plot.XData = x_pos(i);
    rocket_plot.YData = y_pos(i);

    rocket_plot_up.XData = x_pos(i) + 40*(sin(phi_pos(i)));
    rocket_plot_up.YData = y_pos(i) + 40*(cos(phi_pos(i)));
    
    % Update the rocket path (leave the trace)
    set(path_plot, 'XData', x_pos(1:i), 'YData', y_pos(1:i));
    
    % Pause for a short time to create the animation effect
    pause(0.001);
end
