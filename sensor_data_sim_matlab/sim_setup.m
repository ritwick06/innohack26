clear; clc; clf;
dt = 0.1;               
v_nominal = 1.5;       
v_collect = 0.6;        
total_mass = 80;        
cat_width = 1.5;        
sweep_width = 2.0;

boundary_x = [0, 25, 25, 0]; 
boundary_y = [0, 0, 40, 40];

trash_pts = [10, 15; 18, 25; 5, 35];
trash_collected = zeros(size(trash_pts, 1), 1);


waypoints = [];
for y = 0:sweep_width:max(boundary_y)
    row = y/sweep_width;
    if mod(row, 2) == 0
        waypoints = [waypoints; min(boundary_x), y; max(boundary_x), y];
    else
        waypoints = [waypoints; max(boundary_x), y; min(boundary_x), y];
    end
end


pos = [0, 0, 0]; 
Kp = 1.8;
trajectory = [];

for i = 1:size(waypoints, 1)
    target = waypoints(i, :);
    
    while norm(target - pos(1:2)) > 0.6
        for t = 1:size(trash_pts, 1)
            dist_to_trash = norm(trash_pts(t,:) - pos(1:2));
            if trash_collected(t) == 0 && dist_to_trash < 4.0
                trash_target = trash_pts(t,:);
                while norm(trash_target - pos(1:2)) > 0.4
                    error_t = atan2(trash_target(2)-pos(2), trash_target(1)-pos(1)) - pos(3);
                    error_t = atan2(sin(error_t), cos(error_t));
                    pos(1) = pos(1) + v_collect * cos(pos(3)) * dt;
                    pos(2) = pos(2) + v_collect * sin(pos(3)) * dt;
                    pos(3) = pos(3) + (Kp * error_t) * dt;
                    trajectory = [trajectory; pos(1:2)];
                end
                trash_collected(t) = 1;
            end
        end

        desired_theta = atan2(target(2) - pos(2), target(1) - pos(1));
        heading_error = atan2(sin(desired_theta - pos(3)), cos(desired_theta - pos(3)));
        
        omega = Kp * heading_error;
        pos(1) = pos(1) + v_nominal * cos(pos(3)) * dt;
        pos(2) = pos(2) + v_nominal * sin(pos(3)) * dt;
        pos(3) = pos(3) + omega * dt;
        
        trajectory = [trajectory; pos(1:2)];
        
        if size(trajectory, 1) > 10000, break; end
    end
end

%%plots
figure(1); hold on;
plot(boundary_x([1:end,1]), boundary_y([1:end,1]), 'k--', 'LineWidth', 2);
plot(waypoints(:,1), waypoints(:,2), 'b--o', 'MarkerSize', 3);
plot(trajectory(:,1), trajectory(:,2), 'r', 'LineWidth', 2);
scatter(trash_pts(:,1), trash_pts(:,2), 100, 'g', 'filled');
title('Autonomous Cleaning Robot Simulation (PS26)');
xlabel('X (meters)'); ylabel('Y (meters)');
legend('Boundary', 'RPi 5 Plan', 'STM32 Actual Path', 'Trash Collected');
grid on; axis equal;