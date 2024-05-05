clc
clear
close all
%%

results = readmatrix("../logging/nodes.txt")

x = results(:,1);
y = results(:,2);

figure
scatter(x,y)