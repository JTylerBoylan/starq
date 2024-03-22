clc
clear
close all
%%

syms phi theta psi

% phi = yaw [z]
% theta = pitch [y]
% psi = roll [x]

M = [cos(theta)*cos(phi) -sin(phi) 0;
     cos(theta)*sin(phi) cos(phi) 0;
     0 0 1];

Minv = simplify(inv(M));

Minv = subs(Minv, theta, 0)

R = @(p) subs(Minv, phi, p);

deul = [-0.177; -0.04; 0.02];

omega = double(R(1.64).'*deul)

deul = double(R(1.64)*omega)