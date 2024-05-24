clc
clear
close all

syms L1_ L2_ YA_
syms A B

thetaA = YA_ * A;
thetaB = YA_ * B;

alpha = 0.5 * (pi - thetaA - thetaB);
gamma = asin(L1_*sin(alpha)/L2_);
phi = pi - alpha - gamma;

theta = thetaA + alpha;
R = L2_*sin(phi)/sin(alpha);

X = R*cos(theta);
Y = R*sin(theta);

P = [X; Y];

J = jacobian(P, [A; B]);

forward_jacobian = simplify(J)

J_11 = forward_jacobian(1,1)
J_12 = forward_jacobian(1,2)
J_21 = forward_jacobian(2,1)
J_22 = forward_jacobian(2,2)