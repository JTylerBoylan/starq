clc
clear
close all

%%

% User-defined constants
horizon_window = 0.5; % seconds
num_nodes = 16;

z_constant = 0.5;

time_stance = 3/4 * horizon_window;

body_length = 0.5;
body_width = 0.25;
mass = 43; % kg
Ixx = 0.41; % kgm2
Iyy = 2.1; % kgm2
Izz = 2.1; % kgm2

FL_idx_offset = 0;
BL_idx_offset = num_nodes * 3/4;
BR_idx_offset = num_nodes * 1/4;
FR_idx_offset = num_nodes * 2/4;

% Control inputs
desired_velocity = 0.5; % m/s
desired_rotation = 0.0; % rad/s
initial_position = [0.0, 0.0, 0.0]; % x y theta [m, rad]

% Conversions
step_length = time_stance * desired_velocity;

r_hip_FL = [+body_length/2; +body_width/2; 0];
r_hip_BL = [-body_length/2; +body_width/2; 0];
r_hip_BR = [-body_length/2; -body_width/2; 0];
r_hip_FR = [+body_length/2; -body_width/2; 0];

I = [Ixx, 0, 0;
     0, Iyy, 0;
     0, 0, Izz];

% Generate reference trajectory
t = linspace(0, horizon_window, num_nodes);

phi_dot_ref = desired_rotation * ones(size(t));
phi_ref = initial_position(3) + phi_dot_ref.*t;

x_dot_ref = desired_velocity * cos(phi_ref);
y_dot_ref = desired_velocity * sin(phi_ref);
x_ref = initial_position(1) + x_dot_ref.*t;
y_ref = initial_position(2) + y_dot_ref.*t;

z_ref = z_constant * ones(size(t));

% Generate foot positions
r_hip = [step_length/2 - desired_velocity*t;
         zeros(size(t));
         -z_constant * ones(size(t))];

% Translate into body frame
r_body_FL = r_hip + r_hip_FL;
r_body_BL = r_hip + r_hip_BL;
r_body_BR = r_hip + r_hip_BR;
r_body_FR = r_hip + r_hip_FR;

% Rotate into world frame
r_FL = nan(3, num_nodes);
r_BL = nan(3, num_nodes);
r_BR = nan(3, num_nodes);
r_FR = nan(3, num_nodes);
for k = 1:num_nodes
    r_FL(:,k) = getRz(phi_ref(k)) * r_body_FL(:,k);
    r_BL(:,k) = getRz(phi_ref(k)) * r_body_BL(:,k);
    r_BR(:,k) = getRz(phi_ref(k)) * r_body_BR(:,k);
    r_FR(:,k) = getRz(phi_ref(k)) * r_body_FR(:,k);
end

% Shift by offset
FL_offset = [FL_idx_offset+1:num_nodes, 1:FL_idx_offset];
BL_offset = [BL_idx_offset+1:num_nodes, 1:BL_idx_offset];
BR_offset = [BR_idx_offset+1:num_nodes, 1:BR_idx_offset];
FR_offset = [FR_idx_offset+1:num_nodes, 1:FR_idx_offset];

r_FL = r_FL(:,FL_offset);
r_BL = r_BL(:,BL_offset);
r_BR = r_BR(:,BR_offset);
r_FR = r_FR(:,FR_offset);

% Get swing feet
in_swing = t > time_stance;

in_swing_FL = in_swing(FL_offset);
in_swing_BL = in_swing(BL_offset);
in_swing_BR = in_swing(BR_offset);
in_swing_FR = in_swing(FR_offset);

% Replace swing positions w/ zero
r_FL(:,in_swing_FL) = 0;
r_BL(:,in_swing_BL) = 0;
r_BR(:,in_swing_BR) = 0;
r_FR(:,in_swing_FR) = 0;

% Get matrices
phi_avg = mean(phi_ref);

Ai = getAi(phi_avg)

Bi = getBi(r_FL(:,1), r_BL(:,1), r_BR(:,1), r_FR(:,1), phi_ref(1), I, mass)


%% Functions

function Rz = getRz(theta)
    Rz = [cos(theta) -sin(theta) 0;
          sin(theta) cos(theta) 0;
          0 0 1];
end

function rx = getSkewMatrix3(r)
    rx = [0 -r(3) r(2);
          r(3) 0 -r(1);
          -r(2) r(1) 0];
end

function Ai = getAi(phi_avg)
    z3 = zeros(3,3);
    o3 = ones(3,3);
    
    Ai = [z3, z3, getRz(phi_avg), z3;
          z3, z3, z3, o3;
          z3, z3, z3, z3;
          z3, z3, z3, z3];
end

function Bi = getBi(r_FL_i, r_BL_i, r_BR_i, r_FR_i, phi_i, I, m)
    z3 = zeros(3,3);
    inv_m3 = ones(3,3) / m;

    Rz = getRz(phi_i);

    I_bar = Rz * I * Rz';

    Irx_FL = I_bar \ getSkewMatrix3(r_FL_i);
    Irx_BL = I_bar \ getSkewMatrix3(r_BL_i);
    Irx_BR = I_bar \ getSkewMatrix3(r_BR_i);
    Irx_FR = I_bar \ getSkewMatrix3(r_FR_i);

    Bi = [z3, z3, z3, z3;
          z3, z3, z3, z3;
          Irx_FL, Irx_BL, Irx_BR, Irx_FR;
          inv_m3, inv_m3, inv_m3, inv_m3];
end



