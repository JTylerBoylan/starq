clc
clear
close all
%%

Rz = @(th) [cos(th) -sin(th) 0 0;
            sin(th) cos(th) 0 0;
            0 0 1 0;
            0 0 0 1];

Ry = @(th) [cos(th) 0 sin(th) 0;
            0 1 0 0;
            -sin(th) 0 cos(th) 0;
            0 0 0 1];

Rx = @(th) [1 0 0 0;
            0 cos(th) -sin(th) 0;
            0 sin(th) cos(th) 0;
            0 0 0 1];

Tx = @(d) [1 0 0 d;
           0 1 0 0;
           0 0 1 0;
           0 0 0 1];

Ty = @(d) [1 0 0 0;
           0 1 0 d;
           0 0 1 0;
           0 0 0 1];

Tz = @(d) [1 0 0 0;
           0 1 0 0;
           0 0 1 d;
           0 0 0 1];

% Unitree A1 Properties
syms d lc lt th1 th2 th3
% syms ox oy

% R_C0 = Rz(0);
% T_C0 = Tx(ox)*Ty(oy);

R_01 = Rx(th1);
T_01 = Ty(d);
A_01 = R_01*T_01;

R_12 = Ry(th2);
T_12 = Tz(-lt);
A_12 = R_12*T_12;
A_02 = A_01*A_12;

R_23 = Ry(th3);
T_23 = Tz(-lc);
A_23 = R_23*T_23;
A_03 = A_02*A_23;

R_03 = A_03(1:3, 1:3);
T_03 = A_03(1:3, 4);

J_03 = simplify(jacobian(T_03, [th1 th2 th3]))
