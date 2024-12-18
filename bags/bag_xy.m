close all
clear all
clc

%% Paths

lin_pol_jnt_path = 'D:\Shared Content\hmwrk4\my_xy_bag';

%% Bag Objects

lin_pol_jnt_bag = ros2bag(lin_pol_jnt_path);

%% Read Messages from Topics

effort_topic = '/xy_bag';

lin_pol_jnt_sel = select(lin_pol_jnt_bag, "Topic", effort_topic);

lin_pol_jnt_msgs = readMessages(lin_pol_jnt_sel);

%% Process and Analyze data

% Extract positions for x and y
lin_pol_jnt_dt_x = cellfun(@(msg) msg.data(1)', lin_pol_jnt_msgs, 'UniformOutput', false);
lin_pol_jnt_dt_y = cellfun(@(msg) msg.data(2)', lin_pol_jnt_msgs, 'UniformOutput', false);

% Convert cell arrays to matrices
lin_pol_jnt_dt_x_array = cell2mat(lin_pol_jnt_dt_x);
lin_pol_jnt_dt_y_array = cell2mat(lin_pol_jnt_dt_y);

% Plot x vs y in the XY plane
figure;
plot(lin_pol_jnt_dt_x_array, lin_pol_jnt_dt_y_array, 'b-', 'LineWidth', 2, 'DisplayName', 'fra2mo trajectory');
hold on;

% Set axis limits
xlim([-3.5 7]);
ylim([-3 5]);

% Add a red fixed point at (-3, 4.5)
plot(-3, 4.5, 'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'r', 'DisplayName', 'Initial point');

% Add green points
green_points = [6.5, -1.4; -1.6, -2.5; 6, 4; 0, 3];
plot(green_points(:, 1), green_points(:, 2), 'go', 'MarkerSize', 4, 'MarkerFaceColor', 'g', 'DisplayName', 'Goal point');

% Add title and labels
title('Trajectory in the XY Plane');
xlabel('X Position');
ylabel('Y Position');
grid on;
axis equal; % Ensure equal scaling for x and y axes

% Add legend
legend('show', 'Location', 'best');

hold off;

