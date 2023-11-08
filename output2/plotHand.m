clc; clear; close all;
addpath("../")
rcsRenderinng = false;
drawCamera = false;
load("push_pull.mat");
if drawCamera == false
    keypoints = handword_keypoints;
end
Njoints = size(keypoints,2);
Nframes = length(timestampList);
frameLength = 1/fps;
HAND_PALM_CONNECTIONS = [1 2; 1, 6; 10 14; 14 18; 6 10; 1 18];
HAND_THUMB_CONNECTIONS = [2 3; 3 4; 4 5];
HAND_INDEX_FINGER_CONNECTIONS = [6 7; 7 8; 8 9];
HAND_MIDDLE_FINGER_CONNECTIONS = [10 11; 11 12; 12, 13];
HAND_RING_FINGER_CONNECTIONS = [14 15; 15 16; 16 17];
HAND_PINKY_FINGER_CONNECTIONS = [18 19; 19 20; 20 21];
HAND_FINGER_CONNECTIONS = [HAND_THUMB_CONNECTIONS; HAND_INDEX_FINGER_CONNECTIONS; HAND_MIDDLE_FINGER_CONNECTIONS; HAND_RING_FINGER_CONNECTIONS; HAND_PINKY_FINGER_CONNECTIONS];
connections = [HAND_PALM_CONNECTIONS; HAND_FINGER_CONNECTIONS];
hf = figure;
hf.Color = 'white';
% for ii = 1:1:length(timestampList)
for ii = 1
    cla
    x = squeeze(keypoints(ii,:,1));
    y = squeeze(keypoints(ii,:,2));
    z = squeeze(keypoints(ii,:,3));
    if rcsRenderinng == true
        hand = plot3(z,x,y,'.','markersize', 20,'Color',"k");
    else
        hand = plot3(z,x,y,'.','markersize', 20,'Color',"b");
    end
    hold on;
    if drawCamera == true
        camera = scatter3(0,0,0,100,"red",'o');
    end
    if rcsRenderinng == false
        for jj = 1:1:size(connections,1)
            plot3(z(connections(jj,:)),x(connections(jj,:)),y(connections(jj,:)),'Color','b','LineWidth',0.5);
        end
    end
    for jj = 1:1:size(connections)
        joint1(1:3) = keypoints(ii,connections(jj,1),1:3);
        joint2(1:3) = keypoints(ii,connections(jj,2),1:3);
        mid = 0.5*(joint1+joint2);
        if rcsRenderinng == true
            aspect = joint1 - joint2;
            a = norm(aspect)/4;
            b = norm(aspect)/4;
            c = norm(aspect)/2;
            [X,Y,Z] = ellipsoid2P(joint1,joint2,a,b,c,10);
            surf(Z,X,Y);
            colormap white;
        end
    end
    xlabel('Z (m)'); ylabel('X (m)'); zlabel('Y (m)'); % title(sprintf('Timestamp: %f (ms)',timestampList(ii)));
    grid on;
    origin = [0, 0, 0];
    % 指定箭头的长度
    arrowLength = 0.05;
    % 指定 x、y、z 坐标轴的方向向量
    xAxis = [arrowLength, 0, 0];
    yAxis = [0, arrowLength, 0];
    zAxis = [0, 0, arrowLength];
    % 绘制 x、y、z 坐标轴箭头
    hold on;
    quiver3(origin(3), origin(1), origin(2), xAxis(3), xAxis(1), xAxis(2), 'r', 'LineWidth', 3);
    quiver3(origin(3), origin(1), origin(2), yAxis(3), yAxis(1), yAxis(2), 'g', 'LineWidth', 3);
    quiver3(origin(3), origin(1), origin(2), zAxis(3), zAxis(1), zAxis(2), 'b', 'LineWidth', 3);
    % 添加文本标签
    if drawCamera == true
        text(xAxis(3) - 0.025, xAxis(1), xAxis(2), 'x', 'FontSize', 18, 'Color', 'r');
        text(yAxis(3), yAxis(1) + 0.025, yAxis(2), 'y', 'FontSize', 18, 'Color', 'g');
        text(zAxis(3), zAxis(1), zAxis(2) + 0.025, 'z', 'FontSize', 18, 'Color', 'b');
        legend([camera hand] ,'Camera','Hand joints','Location','northwest');
    else
        text(xAxis(3) - 0.01, xAxis(1), xAxis(2), 'x', 'FontSize', 18, 'Color', 'r');
        text(yAxis(3), yAxis(1) + 0.01, yAxis(2), 'y', 'FontSize', 18, 'Color', 'g');
        text(zAxis(3), zAxis(1), zAxis(2) + 0.01, 'z', 'FontSize', 18, 'Color', 'b');
    end
    set(gca,'ZDir','reverse'); %Y
    axis equal;
    view(-60,30)
    %     set(gcf,'units','normalized','outerposition',[0 0 1 1]);
    drawnow;
    set(gca, 'FontSize', 18);
end
% tightfig