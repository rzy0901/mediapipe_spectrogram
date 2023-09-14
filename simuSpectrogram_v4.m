% only for drawing imgs for paper
function simuSpectrogram_v4(Tx_pos,Rx_pos,fc,fs,AWGN_mean,AWGN_var,thres_A_TRD, ...
    drawScenario,rcsRenderinng,input_mat_path,using_camera_coordinate,connections, ...
    output_jpg_path,output_gif_path,pic_save)
% This codes could simulate spectrogram of a 21 keypoint hand using primitive based method.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input:
% Tx_pos: transmitter positions in meters
% Rx_pos: receiver positions in meters.
% fx: carrier frequency in Hz.
% fs: new frame rate after interpolations.
% AWGN_mean: mean value for RCS noises in squared meters (DC component, approximation to targert-unralted rays).
% AWGN_mean: variance for RCS noises in squared meters.
% thres_A_TRD: threshold value for amplitude of time(range) doppler spectrogram.
% drawScenario: whether draw the simulation scenario animation.
% rcsRenderinng: whether render the 3d hand keypoints in the animation.
% input_mat_path: input full file path for 3d hand keypoints `.mat` file.
% using_camera_coordinate: true for hand joints in camera coordinate and
% false for hand joints in hand world coordinate (no translations).
% connections: Regoin of interest for hand bone connections.
% output_jpg_path: output full file path for spectrogram figure.
% output_gif_path: output full file path for
% pic_save: whether save the generated result to disk.
% Output:
% Simulation gifs and spectrogram figures.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
arguments
    Tx_pos (1,3) double
    Rx_pos (1,3) double
    fc (1,1) double
    fs (1,1) double
    AWGN_mean (1,1) double
    AWGN_var (1,1) double
    thres_A_TRD (1,1) double
    drawScenario (1,1) logical
    rcsRenderinng (1,1) logical
    input_mat_path  (1,1) string
    using_camera_coordinate (1,1) logical
    connections (:,2) double
    output_jpg_path (1,1) string
    output_gif_path (1,1) string
    pic_save (1,1) logical
end

load(input_mat_path);

if ~using_camera_coordinate
    keypoints = handword_keypoints;
end

Njoints = size(keypoints,2);
Nframes = length(timestampList);
frameLength = 1/fps;

T = frameLength*Nframes;
fontsize = 16;
%% plot
if drawScenario == true
    hf = figure;
    hf.Color = 'white';
    %     for ii = 1:1:length(timestampList)
    for ii = 1
        cla
        x = squeeze(keypoints(ii,:,1));
        y = squeeze(keypoints(ii,:,2));
        z = squeeze(keypoints(ii,:,3));
        % plot
        if rcsRenderinng == true
            hand = plot3(z,x,y,'.','markersize', 20,'Color',"k");

        else
            hand = plot3(z,x,y,'.','markersize', 13,'Color',"b");
        end
        hold on;
        axis equal;
        view(-80,4)
        if using_camera_coordinate
            camera = scatter3(0,0,0,200,"red",'o');
        else
            hand_center = scatter3(0,0,0,200,"red",'o');
        end
        tx = scatter3(Tx_pos(3),Tx_pos(1),Tx_pos(2),200,"magenta",'*');
        rx = scatter3(Rx_pos(3),Rx_pos(1),Rx_pos(2),200,"black",'*');
        if rcsRenderinng == false
            for jj = 1:1:size(connections,1)
                %             plot3(z(connections(jj,:)),x(connections(jj,:)),y(connections(jj,:)),'Color','b','LineWidth',0.05);
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
            line([Tx_pos(3) mid(3)],[Tx_pos(1) mid(1)],...
                [Tx_pos(2) mid(2)],'LineStyle',':',...
                'color',[0.5 0.5 0.5],'LineWidth',1)
            line([Rx_pos(3) mid(3)],[Rx_pos(1) mid(1)],...
                [Rx_pos(2) mid(2)],'LineStyle',':',...
                'color',[0.5 0.5 0.5],'LineWidth',1)
        end
        xlabel('Z(m)','FontSize',fontsize','Interpreter', 'latex'); ylabel('X(m)','FontSize',fontsize,'Interpreter', 'latex'); zlabel('Y(m)','FontSize',fontsize,'Interpreter', 'latex');
        %         title(sprintf('Timestamp: %f (ms)',timestampList(ii)));
        grid on;
        set(gca,'ZDir','reverse'); %Y
        set(gcf,'units','normalized','outerposition',[0 0 1 1]);
        arrowLength = 0.05;
        origin = [0, 0, 0];
        % 指定 x、y、z 坐标轴的方向向量
        xAxis = [arrowLength, 0, 0];
        yAxis = [0, arrowLength, 0];
        zAxis = [0, 0, arrowLength];
        % 绘制 x、y、z 坐标轴箭头
        hold on;
        quiver3(origin(3), origin(1), origin(2), xAxis(3), xAxis(1), xAxis(2), 'r', 'LineWidth', 3);
        quiver3(origin(3), origin(1), origin(2), yAxis(3), yAxis(1), yAxis(2), 'g', 'LineWidth', 3);
        quiver3(origin(3), origin(1), origin(2), zAxis(3), zAxis(1), zAxis(2), 'b', 'LineWidth', 3);
        drawnow;
        ax = gca;
        ax.XAxis.FontSize = fontsize;
        ax.YAxis.FontSize = fontsize;
        ax.ZAxis.FontSize = fontsize;
        ax.XAxis.TickLabelInterpreter = 'latex';
        ax.YAxis.TickLabelInterpreter = 'latex';
        ax.ZAxis.TickLabelInterpreter = 'latex';
        if using_camera_coordinate
            %             legend([camera tx rx hand] ,'camera','transmitter','receiver','hand joints','FontSize',18);
            lgd = legend([camera tx rx hand] ,'origin','transmitter','receiver','hand joints','FontSize',fontsize,'Location','NorthWest','Interpreter', 'latex');
            lgd.NumColumns = 2;
            lgd.MarkerSize = fontsize;
        else
            %             legend([hand_center tx rx hand] ,'hand center','transmitter','receiver','hand joints');
        end

        %         if pic_save == true
        %             Frame=getframe(gcf);
        %             Image=frame2im(Frame);
        %             [Image,map]=rgb2ind(Image,256);
        %             if ii == 1
        %                 imwrite(Image,map,output_gif_path,'gif', 'Loopcount',inf,'DelayTime',1/fps);
        %             else
        %                 imwrite(Image,map,output_gif_path,'gif','WriteMode','append','DelayTime',1/fps);
        %             end
        %         end
    end
end
end