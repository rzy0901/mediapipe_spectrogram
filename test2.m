clc; clear; close all;
P1 = [0, 0, -5];
P2 = [2, 2, 2];
c = norm(P1-P2)/2;
a = c / 2;
b = c / 2; 
tx = scatter3(P1(1),P1(2),P1(3),100,"magenta",'*');
hold on;
rx = scatter3(P2(1),P2(2),P2(3),100,"black",'*');
[X,Y,Z] = ellipsoid2P_transformed(P1,P2,a,b,c,11);
surf(X,Y,Z);
xlabel('x')
ylabel('y')
zlabel('z')
% xlim([0 4])
% ylim([0 4])
% zlim([0 4])
colormap white;
axis equal;