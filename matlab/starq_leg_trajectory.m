clc
clear
close all
%%

traj_dat = readmatrix("latestFootSpace.csv", "Delimiter", ',');

sz = size(traj_dat(:,1));

t1 = traj_dat(:,1) * 10;
id1 = 0*ones(sz);
cm1 = 3*ones(sz);
im1 = 1*ones(sz);
x1 = traj_dat(:,2) * 1E-3;
y1 = traj_dat(:,3) * 1E-3;
z1 = zeros(sz);
v1 = zeros(sz(1), 3);
f1 = zeros(sz(1), 3);

t2 = traj_dat(:,1) * 10;
id2 = 1*ones(sz);
cm2 = 3*ones(sz);
im2 = 1*ones(sz);
x2 = [x1(1:50); x1(51:end)];
y2 = [y1(1:50); y1(51:end)];
z2 = zeros(sz);
v2 = zeros(sz(1), 3);
f2 = zeros(sz(1), 3);

t3 = traj_dat(:,1) * 10;
id3 = 2*ones(sz);
cm3 = 3*ones(sz);
im3 = 1*ones(sz);
x3 = x1;
y3 = y1;
z3 = zeros(sz);
v3 = zeros(sz(1), 3);
f3 = zeros(sz(1), 3);

t4 = traj_dat(:,1) * 10;
id4 = 3*ones(sz);
cm4 = 3*ones(sz);
im4 = 1*ones(sz);
x4 = x2;
y4 = y2;
z4 = zeros(sz);
v4 = zeros(sz(1), 3);
f4 = zeros(sz(1), 3);

t = [t1; t2; t3; t4];
id = [id1; id2; id3; id4];
cm = [cm1; cm2; cm3; cm4];
im = [im1; im2; im3; im4];
x = [x1; x2; x3; x4];
y = [y1; y2; y3; y4];
z = [z1; z2; z3; z4];
p = [x, y, z];
v = [v1; v2; v3; v4];
f = [f1; f2; f3; f4];

out = [t, id, cm, im, p, v, f]

writematrix(out, '../starq/trajectories/walk_test.txt', 'Delimiter', ' ');
