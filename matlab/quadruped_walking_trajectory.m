clc
clear
close all
%%

% Params
stride_frequency = 1; % Hz
stride_length = 0.050; % mm
publish_frequency = 100; % Hz

N = publish_frequency / stride_frequency;
t = linspace(0, 1/stride_frequency, N);

z_nom = -0.25;
x_land = 0.25;
x_lift = -0.25;
z_lift = 0.10;

x_stance = x_land + t * (x_lift - x_land) / (3/4 * t(end));
x_swing = x_lift + t * (x_land - x_lift) / (1/4 * t(end));

z_stance = z_nom * ones(1, 3/4*N);
z_swing = z_nom + z_lift * sin(4 * pi * t);

x = [x_stance(1:3/4*N), x_swing(1:1/4*N)];
y = zeros(1,N);
z = [z_stance(1:3/4*N), z_swing(1:1/4*N)];

pos_traj = [x; y; z];

figure
title("Position Trajectory")
plot(pos_traj(1,:), pos_traj(3,:))
xlabel("X (mm)")
ylabel("Z (mm)")
axis equal

vel_traj = [diff(pos_traj(1,:))./diff(t), 0;
            diff(pos_traj(2,:))./diff(t), 0;
            diff(pos_traj(3,:))./diff(t), 0];

figure
title("Velocity Trajectory")
plot(vel_traj(1,:), vel_traj(3,:))
xlabel("Vx (mm/s)")
ylabel("Vz (mm/s)")
axis equal

force_traj = zeros(size(pos_traj));

sz = size(t);
leg_id = zeros(sz);
control_mode = 3*ones(sz);
input_mode = 1*ones(sz);
pos_traj_2D = pos_traj([1 3], :);
vel_traj_2D = vel_traj([1 3], :);
force_traj_2D = force_traj([1 3], :);

output = [t', leg_id', control_mode', input_mode', ...
          pos_traj_2D', vel_traj_2D', force_traj_2D']

writematrix(output, '../starq/trajectories/walking.txt', 'Delimiter', ' ');


