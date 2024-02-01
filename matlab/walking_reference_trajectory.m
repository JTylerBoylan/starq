clc
clear
close all

%%

% params
num_strides = 2; % constant
steps_per_stride = 4; % constant
nodes_per_step = 4; % constant
stride_length = 0.25; % constant

% desired speeds
vel_des = 0.5; % variable
rot_des = 0.0; % variable

% conversions
stride_frequency = vel_des / stride_length;
stride_duration = 1/stride_frequency;
total_duration = num_strides * stride_duration;
distance_traveled = total_duration * vel_des;

num_steps = num_strides * steps_per_stride;
num_nodes = num_steps * nodes_per_step;

% initial position
pos_0 = [0.0; 0.0]; % variable 
theta_0 = 0.0; % variable

% robot size
robot_hip_width = 0.5; % constant
robot_hip_length = 1.0; % constant

% hip locations
pos_FL = [+robot_hip_length/2 ; +robot_hip_width/2];
pos_BL = [-robot_hip_length/2 ; +robot_hip_width/2];
pos_BR = [-robot_hip_length/2 ; -robot_hip_width/2];
pos_FR = [+robot_hip_length/2 ; -robot_hip_width/2];

% footfall pattern
stance_foot_pattern = [0 1 1 1;
                       1 1 1 0;
                       1 0 1 1;
                       1 1 0 1]; % constant

% time foot on ground
stance_duration = stride_duration * 3/4; % constant

% COM reference position
t = linspace(0, total_duration, num_nodes);
theta_COM = theta_0 + rot_des * t;
pos_COM = pos_0 + [vel_des * t .* cos(theta_COM);
                   vel_des * t .* sin(theta_COM)];

% COM reference velocity
vel_COM = [diff(pos_COM(1,:))./diff(t);
           diff(pos_COM(2,:))./diff(t)];
vel_COM = [vel_COM, vel_COM(:,end)];

% 2D rotation
Rot = @(theta) [cos(theta) -sin(theta);
                sin(theta) cos(theta)];

% Foot placement
step_idx = 1;
node_idx = 1;
pos_FL_i = pos_FL(:,1);
pos_BL_i = pos_BL(:,1);
pos_BR_i = pos_BR(:,1);
pos_FR_i = pos_FR(:,1);
pos_FL_traj = nan(2, num_nodes);
pos_BL_traj = nan(2, num_nodes);
pos_BR_traj = nan(2, num_nodes);
pos_FR_traj = nan(2, num_nodes);
for std = 1:num_strides

    for stp = 1:steps_per_stride

        % current state
        pos_COM_i = pos_COM(:, node_idx);
        theta_COM_i = theta_COM(:, node_idx);
        vel_COM_i = vel_COM(:, node_idx);

        % current rotation
        Rot_i = Rot(theta_COM_i);

        % feet in stance
        stance_feet = stance_foot_pattern(:, stp);

        % last feet in stance
        if (stp ~= 1)
            last_stance_feet = stance_foot_pattern(:, stp-1);
        else
            last_stance_feet = stance_foot_pattern(:, end);
        end

        % landing foot if transitioning to stance
        land_feet = stance_feet & ~last_stance_feet;

        % update position on land
        if (land_feet(1))
            pos_FL_i = pos_COM_i + Rot_i*pos_FL + vel_COM_i * stance_duration / 2;
        end
        if (land_feet(2))
            pos_BL_i = pos_COM_i + Rot_i*pos_BL + vel_COM_i * stance_duration / 2;
        end
        if (land_feet(3))
            pos_BR_i = pos_COM_i + Rot_i*pos_BR + vel_COM_i * stance_duration / 2;
        end
        if (land_feet(4))
            pos_FR_i = pos_COM_i + Rot_i*pos_FR + vel_COM_i * stance_duration / 2;
        end

        for nd = 1:nodes_per_step
            
            % set node position
            % ignore if in swing
            if (stance_feet(1))
                pos_FL_traj(:,node_idx) = pos_FL_i;
            end
            if (stance_feet(2))
                pos_BL_traj(:,node_idx) = pos_BL_i;
            end
            if (stance_feet(3))
                pos_BR_traj(:,node_idx) = pos_BR_i;
            end
            if (stance_feet(4))
                pos_FR_traj(:,node_idx) = pos_FR_i;
            end

            node_idx = node_idx + 1;
        end
        step_idx = step_idx + 1;
    end
end

% foot positions w.r.t COM
r_FL = pos_FL_traj - pos_COM;
r_BL = pos_BL_traj - pos_COM;
r_BR = pos_BR_traj - pos_COM;
r_FR = pos_FR_traj - pos_COM;


% plot
figure
hold on
plot(t, pos_COM(1,:), 'o')
plot(t, pos_FL_traj(1,:), 'x')
plot(t, pos_BL_traj(1,:), 'x')
plot(t, pos_BR_traj(1,:), 'x')
plot(t, pos_FR_traj(1,:), 'x')


robot_stand_height = 0.5;
robot_height = 0.25;

% animate
anim = figure();
hold on
FL_plot = plot(0,0);
BL_plot = plot(0,0);
BR_plot = plot(0,0);
FR_plot = plot(0,0);
fill([pos_BR(1) pos_BR(1) pos_FR(1) pos_FR(1)], [0 robot_height robot_height 0], 'k')
axis([-0.75 0.75 -0.75 0.3])
for k = 1:num_nodes
    figure(anim)

    set(FL_plot, 'xdata', [pos_FL(1), r_FL(1, k)], 'ydata', [0, -robot_stand_height]);
    set(BL_plot, 'xdata', [pos_BL(1), r_BL(1, k)], 'ydata', [0, -robot_stand_height]);
    set(BR_plot, 'xdata', [pos_BR(1), r_BR(1, k)], 'ydata', [0, -robot_stand_height]);
    set(FR_plot, 'xdata', [pos_FR(1), r_FR(1, k)], 'ydata', [0, -robot_stand_height]);

    pause(3 * total_duration / num_nodes)

end
