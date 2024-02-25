clc
clear
close all
%%

syms q1 q2 q3

% Rotation functions
Rz = @(th) [cos(th) -sin(th) 0;
            sin(th) cos(th) 0;
            0 0 1 ];

Ry = @(th) [cos(th) 0 sin(th);
            0 1 0;
            -sin(th) 0 cos(th)];

Rx = @(th) [1 0 0;
            0 cos(th) -sin(th);
            0 sin(th) cos(th)];

% Body relative positions
syms d lt lc

b_r_ab = [0; d; 0];
c_r_bc = [0; 0; -lt];
d_r_cd = [0; 0; -lc];

% Reference frames
a_R_b = Rx(q1);
b_R_c = Ry(q2);
c_R_d = Ry(q3);

r_a = [0; 0; 0];
r_b = r_a + a_R_b * b_r_ab;
r_c = r_b + a_R_b * b_R_c * c_r_bc;
r_d = r_c + a_R_b * b_R_c * c_R_d * d_r_cd;

% Rotations
R_1 = a_R_b;
R_2 = R_1 * b_R_c;
R_3 = R_2 * c_R_d;

% Jacobian
q = [q1; q2; q3];

Jee = jacobian(r_d, q);
Jee = simplify(Jee)