clc
clear
close all
%%

input_file = "ONRTrudge60PerFootSpace.csv";
output_file = "trudge_test_1.txt";
offsets = [0 0.5 0 0.5];

traj_dat = readmatrix(input_file, "Delimiter", ',');
sz = size(traj_dat(:,1), 1);
t = floor(linspace(0, 1E3, sz)).';
output = zeros(sz, 13);

for i = 1:length(offsets)

    r0 = (i - 1) * sz + 1;
    r = r0:(r0 + sz - 1);

    off = offsets(i);
    shift = [sz*off+1:sz, 1:sz*off];

    output(r, 1) = t; % time
    output(r, 2) = i - 1; % leg id
    output(r, 3) = 3; % control mode
    output(r, 4) = 1; % input mode
    output(r, 5) = traj_dat(shift, 2) * 1E-3; % x
    % skip y = 0
    output(r, 7) = traj_dat(shift, 3) * 1E-3; % z
    % skip vx, vy, vz = 0
    % skip fx, fy, fz = 0


end

writematrix(output, strcat('../starq/trajectories/', output_file), 'Delimiter', ' ');
