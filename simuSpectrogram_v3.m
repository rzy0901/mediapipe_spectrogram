%% Only for test.
function simuSpectrogram_v3(Tx_pos,Rx_pos,fc,fs,AWGN_mean,AWGN_var,thres_A_TRD, ...
    drawScenario,rcsRenderinng,input_mat_path,using_camera_coordinate,connections, ...
    output_jpg_path,output_gif_path,pic_save)
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
%% plot
if drawScenario == true
    hf = figure;
    hf.Color = 'white';
    for ii = 1
        cla
        x = squeeze(keypoints(ii,:,1));
        y = squeeze(keypoints(ii,:,2));
        z = squeeze(keypoints(ii,:,3));
        % plot
        if rcsRenderinng == true
            hand = plot3(z,x,y,'.','markersize', 40,'Color',"k");

        else
            hand = plot3(z,x,y,'.','markersize', 13,'Color',"b");
        end
        hold on;
        axis equal;
        axis off;
        xmin = min([min(keypoints(:,:,1),[],'all')]);
        xmax = max([max(keypoints(:,:,1),[],'all')]);
        ymin = min([min(keypoints(:,:,2),[],'all')]);
        ymax = max([max(keypoints(:,:,2),[],'all')]);
        zmin = min([min(keypoints(:,:,3),[],'all')]);
        zmax = max([max(keypoints(:,:,3),[],'all')]);
%         xlim([zmin-0.01 zmax+0.01]); % Z
%         ylim([xmin-0.01 xmax+0.01]); % X
%         zlim([ymin-0.01 ymax+0.01]); % Y
        view(-80,-20)
        for jj = 1:1:size(connections,1)
            %             plot3(z(connections(jj,:)),x(connections(jj,:)),y(connections(jj,:)),'Color','b','LineWidth',0.05);
%             plot3(z(connections(jj,:)),x(connections(jj,:)),y(connections(jj,:)),'Color','b','LineWidth',0.5);
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
        xlabel('Z(m)'); ylabel('X(m)'); zlabel('Y(m)'); % title(sprintf('Timestamp: %f (ms)',timestampList(ii)));
        set(gca,'ZDir','reverse'); %Y
        tightfig
%         set(gcf,'units','normalized','outerposition',[0 0 1 1]);
        drawnow;
    end
end