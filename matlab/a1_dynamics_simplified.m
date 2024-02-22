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

syms yc1
b_rcom_a1 = [0; yc1; 0];

syms zc2
c_rcom_b2 = [0; 0; zc2];

syms zc3
d_rcom_c3 = [0; 0; zc3];

r_com_1 = r_a + R_1 * b_rcom_a1;
r_com_2 = r_b + R_2 * c_rcom_b2;
r_com_3 = r_c + R_3 * d_rcom_c3;

omega_a1 = [q1; 0; 0];
omega_b2 = [0; q2; 0];
omega_c3 = [0; q3; 0];

omega_1 = omega_a1;
omega_2 = omega_1 + omega_b2;
omega_3 = omega_2 + omega_c3;

% Body inertia
syms m1 Ix1 Iy1 Iz1
m_1 = m1;
I_1 = diag([Ix1 Iy1 Iz1]);
Mx_1 = [diag([m_1 m_1 m_1]), zeros(3);
       zeros(3), I_1];

syms m2 Ix2 Iy2 Iz2
m_2 = m2;
I_2 = diag([Ix2 Iy2 Iz2]);
Mx_2 = [diag([m_2 m_2 m_2]), zeros(3);
       zeros(3), I_2];

syms m3 Ix3 Iy3 Iz3
m_3 = m3;
I_3 = diag([Ix3 Iy3 Iz3]);
Mx_3 = [diag([m_3 m_3 m_3]), zeros(3);
       zeros(3), I_3];

% Jacobians
q = [q1; q2; q3];

Jv_1 = jacobian(r_com_1, q);
Jv_2 = jacobian(r_com_2, q);
Jv_3 = jacobian(r_com_3, q);

Jw_1 = jacobian(omega_1, q);
Jw_2 = jacobian(omega_2, q);
Jw_3 = jacobian(omega_3, q);

Jv_1 = simplify(Jv_1);
Jv_2 = simplify(Jv_2);
Jv_3 = simplify(Jv_3);
Jw_1 = simplify(Jw_1);
Jw_2 = simplify(Jw_2);
Jw_3 = simplify(Jw_3);
R_1 = simplify(R_1);
R_2 = simplify(R_2);
R_3 = simplify(R_3);

J_1 = [Jv_1; Jw_1];
J_2 = [Jv_2; Jw_2];
J_3 = [Jv_3; Jw_3];

M_i = @(m,I,R,Jv,Jw) m*Jv'*Jv + Jw'*R*I*R'*Jw;

M_1 = M_i(m1,I_1,R_1,Jv_1,Jw_1);
M_2 = M_i(m2,I_2,R_2,Jv_2,Jw_2);
M_3 = M_i(m3,I_3,R_3,Jv_3,Jw_3);

M = M_1 + M_2 + M_3
