% https://zhuanlan.zhihu.com/p/422696920
clc; clear; close all;
Tx_pos = [5 5 5];
Rx_pos = [-5 5 5];
joint1 = [1 2 1];
joint2 = [2 3 4];
mid = 0.5*(joint1 + joint2);
aspect = joint1 - joint2;
% 计算经过mid的垂直于aspect的平面P
% 计算平面P的法向量
normal_vector = aspect / norm(aspect);

% 计算Tx_pos与Rx_pos在上述平面P的投影坐标
Tx_projection = Tx_pos - dot(Tx_pos - mid, normal_vector) * normal_vector;
Rx_projection = Rx_pos - dot(Rx_pos - mid, normal_vector) * normal_vector;

% 计算向量Tx_pos - mid 与向量Rx_pos - mid的夹角在上述平面P上投影的弧度大小
angle_rad = acos(dot(Tx_projection - mid, Rx_projection - mid) / (norm(Tx_projection - mid) * norm(Rx_projection - mid)));

dot(Tx_projection-mid,normal_vector)
dot(normal_vector,Tx_projection)-dot(normal_vector,mid)

%%
scatter3(Tx_pos(1), Tx_pos(2), Tx_pos(3), 100, 'r', 'filled');
hold on;
scatter3(Rx_pos(1), Rx_pos(2), Rx_pos(3), 100, 'b', 'filled');
scatter3(mid(1), mid(2), mid(3), 100, 'g', 'filled');
[x_plane, y_plane] = meshgrid(-10:10, -10:10);
% Ax + By + Cz + D = 0;
A = normal_vector(1); B = normal_vector(2); C = normal_vector(3);
D = - dot(normal_vector,mid);
z_plane = (-A * x_plane - B * y_plane - D) / C;
surf(x_plane, y_plane, z_plane, 'FaceAlpha', 0.3, 'FaceColor', 'c', 'EdgeColor', 'none');
scatter3(Tx_projection(1), Tx_projection(2), Tx_projection(3), 100, 'm', 'filled');
scatter3(Rx_projection(1), Rx_projection(2), Rx_projection(3), 100, 'y', 'filled');
xlabel('X');
ylabel('Y');
zlabel('Z');
legend('Tx\_pos', 'Rx\_pos', 'mid', 'Plane P', 'Tx\_projection', 'Rx\_projection');
text(mid(1), mid(2), mid(3), ['\angle = ', num2str(angle_rad, '%.2f')], 'Color', 'k', 'FontSize', 12);
grid on;
axis equal;
hold off;
