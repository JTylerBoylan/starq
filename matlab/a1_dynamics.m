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
% d = 0.08505;
% lt = 0.02;
% lc = 0.02; 

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

b_rcom_a1 = [-0.003311; 0.000635; 3.1e-05];
c_rcom_b2 = [-0.003237; -0.022327; -0.027326];
d_rcom_c3 = [0.00472659; 0; -0.131975];

r_com_1 = r_a + a_R_b * b_rcom_a1;
r_com_2 = r_b + a_R_b * b_R_c * c_rcom_b2;
r_com_3 = r_c + a_R_b * b_R_c * c_R_d * d_rcom_c3;

omega_a1 = [q1; 0; 0];
omega_b2 = [0; q2; 0];
omega_c3 = [0; q3; 0];

omega_1 = omega_a1;
omega_2 = omega_1 + omega_b2;
omega_3 = omega_2 + omega_c3;

% Body inertia
m_1 = 0.696;
Ip_1 = [0.000807752 0.00055293 0.000468983]; 
RIq_1 = [0.494499 0.491507 0.506268 0.507528];
RI_1 = quat2rotm(RIq_1);
I_1 = RI_1*diag(Ip_1)*RI_1';
Mx_1 = [diag([m_1 m_1 m_1]), zeros(3);
       zeros(3), I_1];

m_2 = 1.013;
Ip_2 = [0.00555739 0.00513936 0.00133944];
RIq_2 = [0.999125 0.00256393 -0.0409531 0.00806091];
RI_2 = quat2rotm(RIq_2);
I_2 = RI_2*diag(Ip_2)*RI_2';
Mx_2 = [diag([m_2 m_2 m_2]), zeros(3);
       zeros(3), I_2];

m_3 = 0.226;
Ip_3 = [0.00340344 0.00339393 3.54834e-05];
RIq_3 = [0.706886 0.017653 0.017653 0.706886];
RI_3 = quat2rotm(RIq_3);
I_3 = RI_3*diag(Ip_3)*RI_3';
Mx_3 = [diag([m_3 m_3 m_3]), zeros(3);
       zeros(3), I_3];

% Jacobians
q = [q1; q2; q3];

Jv_1 = simplify(jacobian(r_com_1, q));
Jv_2 = simplify(jacobian(r_com_2, q));
Jv_3 = simplify(jacobian(r_com_3, q));

Jw_1 = simplify(jacobian(omega_1, q));
Jw_2 = simplify(jacobian(omega_2, q));
Jw_3 = simplify(jacobian(omega_3, q));

J_1 = [Jv_1; Jw_1];
J_2 = [Jv_2; Jw_2];
J_3 = [Jv_3; Jw_3];

M = J_1'*Mx_1*J_1 + J_2'*Mx_2*J_2 + J_3'*Mx_3*J_3;
M = simplify(M)

Jv_ee = simplify(jacobian(r_d, q));
