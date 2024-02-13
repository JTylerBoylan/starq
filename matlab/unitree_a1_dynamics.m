clc
clear
close all
%%

syms q1 q2 q3 lc lt ox oy d

g = [ox - lc*sin(q2+q3) - lt*sin(q2);
     oy + d*cos(q1) + lt*cos(q2)*sin(q1) + lc*sin(q1)*cos(q2+q3);
     d*sin(q1) - lt*cos(q1)*cos(q2) - lc*cos(q1)*cos(q2+q3)];

J = jacobian(g, [q1; q2; q3])

% syms x y z
% eq(1) = x == g(1);
% eq(2) = y == g(2);
% eq(3) = z == g(3);
% eq(4) = x^2 + y^2 + z^2 <= lc^3 + lt^2;
% 
% s = solve(eq, [q1 q2 q3]);