load('collected_data (1).mat');  

% Calculate velocities and accelerations from position data
dt = Ts; % Sampling time
num_samples = length(pos_x);

% Generate a time vector
time = (0:num_samples-1) * Ts; % Time in seconds

% Approximate velocities
vel_x = [0; diff(pos_x) / dt]; % Velocity in X
vel_y = [0; diff(pos_y) / dt]; % Velocity in Y
vel_magnitude = sqrt(vel_x.^2 + vel_y.^2); % Total velocity magnitude

% Approximate accelerations
acc_x = [0; diff(vel_x) / dt]; % Acceleration in X
acc_y = [0; diff(vel_y) / dt]; % Acceleration in Y
acc_magnitude = sqrt(acc_x.^2 + acc_y.^2); % Total acceleration magnitude

% Initialize figure with subplots
figure;

% Trajectory animation (subplot 1)
subplot(2, 2, 1);
plotHandle = animatedline('Color', 'r', 'LineWidth', 2); % Animated trajectory line
axis equal;
grid on; hold on;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Trajectory Animation');
robotMarker = plot(nan, nan, 'ko', 'MarkerFaceColor', 'b', 'MarkerSize', 6);

% Adjust zoom level for trajectory animation
buffer = 0.1; % Adjust buffer to control zoom
x_limits = [min(pos_x) - buffer, max(pos_x) + buffer];
y_limits = [min(pos_y) - buffer, max(pos_y) + buffer];
axis([x_limits, y_limits]); % Fix trajectory animation axis limits

% Position plot (subplot 2)
subplot(2, 2, 2);
positionPlot = plot(nan, nan, 'g', 'LineWidth', 1.5, 'DisplayName', 'Position');
hold on;
%plot(pos_x, pos_y, 'k--', 'DisplayName', 'Full Trajectory');
grid on;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Position');
legend;

% Velocity plot (subplot 3)
subplot(2, 2, 3);
velocityPlot = plot(nan, nan, 'b', 'LineWidth', 1.5, 'DisplayName', 'Velocity');
hold on;
%plot(time, vel_magnitude, 'k--', 'DisplayName', 'Velocity Magnitude');
grid on;
xlabel('Time (s)'); % Time in seconds
ylabel('Velocity (m/s)');
title('Velocity');
legend;

% Yaw and acceleration plot (subplot 4)
subplot(2, 2, 4);
yawPlot = plot(nan, nan, 'm', 'LineWidth', 1.5, 'DisplayName', 'Yaw (deg)');
hold on;
accelerationPlot = plot(nan, nan, 'c', 'LineWidth', 1.5, 'DisplayName', 'Acceleration');
grid on;
xlabel('Time (s)'); % Time in seconds
ylabel('Yaw (deg) / Acceleration (m/s^2)');
title('Yaw and Acceleration');
legend;

% Animation loop
for i = 1:num_samples
    % Update trajectory animation
    subplot(2, 2, 1);
    addpoints(plotHandle, pos_x(i), pos_y(i));
    set(robotMarker, 'XData', pos_x(i), 'YData', pos_y(i));
    
    % Update position plot
    subplot(2, 2, 2);
    set(positionPlot, 'XData', pos_x(1:i), 'YData', pos_y(1:i));
    
    % Update velocity plot
    subplot(2, 2, 3);
    set(velocityPlot, 'XData', time(1:i), 'YData', vel_magnitude(1:i));
    
    % Update yaw and acceleration plot
    subplot(2, 2, 4);
    set(yawPlot, 'XData', time(1:i), 'YData', orientation(1:i));
    set(accelerationPlot, 'XData', time(1:i), 'YData', acc_magnitude(1:i));
    
    % Force MATLAB to update the plots
    drawnow;
    pause(Ts); % Simulate real-time motion
end

disp('Animation Complete!');
