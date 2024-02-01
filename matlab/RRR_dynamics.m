clc
clear
close all
%%

% Symbolic variables
syms Xe Ye Ze
syms thA thB thC
syms L1 L2 L3

% Local vectors
w_A = [0; 0; 0];
a_B_A = [0; L1; 0];
b_C_B = [L2; 0; 0];
c_D_C = [L3; 0; 0];

% World -> A frame: Rotation about x axis
w_Rot_a = [1 0 0; 0 cos(thA) -sin(thA); 0 sin(thA) cos(thA)];

% A -> B frame: Rotation about y axis (inverted)
a_Rot_b = [cos(thB) 0 -sin(thB); 0 1 0; sin(thB) 0 cos(thB)];

% B -> C frame: Rotation about y axis (inverted)
b_Rot_c = [cos(thC) 0 -sin(thC); 0 1 0; sin(thC) 0 cos(thC)];

% B in world frame
w_B = w_A + w_Rot_a * a_B_A;

% C in world frame
w_C = w_B + w_Rot_a * a_Rot_b * b_C_B;

% D in world frame (end effector)
w_D = w_C + w_Rot_a * a_Rot_b * b_Rot_c * c_D_C

% % Solve IK
% eq(1) = Xe == w_D(1);
% eq(2) = Ye == w_D(2);
% eq(3) = Ze == w_D(3);
% 
% s = solve(eq, [thA, thB, thC]);

J = jacobian(w_D, [thA; thB; thC])