clc
clear
close all
%%

% Params
stride_frequency = 0.25; % Hz
stride_length = 0.050; % mm
publish_frequency = 100; % Hz

N = publish_frequency / stride_frequency;
t = linspace(0, 1/stride_frequency, N);

z_nom = -0.25;
x_land = 0.15;
x_lift = -0.15;
z_lift = 0.05;

x_stance = x_land + t * (x_lift - x_land) / (3/4 * t(end));
x_swing = x_lift + t * (x_land - x_lift) / (1/4 * t(end));

z_stance = z_nom * ones(1, 3/4*N);
z_swing = z_nom + z_lift * sin(4 * pi * t * stride_frequency);

x = [x_stance(1:3/4*N), x_swing(1:1/4*N)];
y = 0.08505 * ones(1,N);
z = [z_stance(1:3/4*N), z_swing(1:1/4*N)];

pos_traj = [x; y; z];

figure
title("Position Trajectory")
plot(pos_traj(1,:), pos_traj(3,:))
xlabel("X (m)")
ylabel("Z (m)")
axis equal

% vel_traj = zeros(size(pos_traj));
vel_traj = [diff(pos_traj(1,:))./diff(t), 0;
            diff(pos_traj(2,:))./diff(t), 0;
            diff(pos_traj(3,:))./diff(t), 0];

figure
title("Velocity Trajectory")
plot(vel_traj(1,:), vel_traj(3,:))
xlabel("Vx (m/s)")
ylabel("Vz (m/s)")
axis equal

force_traj = zeros(size(pos_traj));

t = floor(t * 1E3);

output = [];
for leg = 0:3

    sz = size(t);
    leg_id = leg * ones(sz);
    control_mode = 3*ones(sz);
    input_mode = 1*ones(sz);

    pos_traj = [pos_traj(:,leg/4*N+1:end), pos_traj(:,1:leg/4*N)];
    vel_traj = [pos_traj(:,leg/4*N+1:end), pos_traj(:,1:leg/4*N)];
    force_traj = [pos_traj(:,leg/4*N+1:end), pos_traj(:,1:leg/4*N)];

    output = [output;
              t', leg_id', control_mode', input_mode', pos_traj', vel_traj', force_traj']

end

writematrix(output, '../starq/trajectories/walking.txt', 'Delimiter', ' ');


