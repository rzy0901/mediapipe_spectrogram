%% Only for test, write mp4 videos.
function simuSpectrogram_v2(Tx_pos,Rx_pos,fc,fs,AWGN_mean,AWGN_var,thres_A_TRD, ...
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

% keypoints = smoothdata(keypoints,1,"rlowess",5);

% %%%%%%%%%% Remove first frame %%%%%%%%%%
% keypoints = keypoints(2:end,:,:);
% timestampList = timestampList(2:end);
% handword_keypoints = handword_keypoints(2:end,:,:);
% transformations = transformations(2:end,:,:);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Njoints = size(keypoints,2);
Nframes = length(timestampList);
frameLength = 1/fps;

T = frameLength*Nframes;
% Tx_pos = [0 -0.1 -1.5]; % XYZ
% Rx_pos = [0 -0.1 0]; % XYZ
% drawScenario = false;
%% plot
if drawScenario == true
    hf = figure;
    hf.Color = 'white';
    for ii = 1:1:length(timestampList)
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
        axis off;
        xmin = min([min(keypoints(:,:,1),[],'all')]);
        xmax = max([max(keypoints(:,:,1),[],'all')]);
        ymin = min([min(keypoints(:,:,2),[],'all')]);
        ymax = max([max(keypoints(:,:,2),[],'all')]);
        zmin = min([min(keypoints(:,:,3),[],'all')]);
        zmax = max([max(keypoints(:,:,3),[],'all')]);
        xlim([zmin-0.01 zmax+0.01]); % Z
        ylim([xmin-0.01 xmax+0.01]); % X
        zlim([ymin-0.01 ymax+0.01]); % Y
        view(-280,10)
        %         view(30,30)
        %         view(2)
        %         view(30,5)
        if using_camera_coordinate
%             camera = scatter3(0,0,0,100,"red",'o');
        else
%             hand_center = scatter3(0,0,0,100,"red",'o');
        end
%         tx = scatter3(Tx_pos(3),Tx_pos(1),Tx_pos(2),100,"magenta",'*');
%         rx = scatter3(Rx_pos(3),Rx_pos(1),Rx_pos(2),100,"black",'*');
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
%             line([Tx_pos(3) mid(3)],[Tx_pos(1) mid(1)],...
%                 [Tx_pos(2) mid(2)],'LineStyle',':',...
%                 'color',[0.5 0.5 0.5],'LineWidth',0.05)
%             line([Rx_pos(3) mid(3)],[Rx_pos(1) mid(1)],...
%                 [Rx_pos(2) mid(2)],'LineStyle',':',...
%                 'color',[0.5 0.5 0.5],'LineWidth',0.05)
        end
        xlabel('Z(m)'); ylabel('X(m)'); zlabel('Y(m)'); % title(sprintf('Timestamp: %f (ms)',timestampList(ii)));
%         grid on;
        if using_camera_coordinate
%             legend([camera tx rx hand] ,'camera','transmitter','receiver','hand joints');
        else
%             legend([hand_center tx rx hand] ,'hand center','transmitter','receiver','hand joints');
        end
        set(gca,'ZDir','reverse'); %Y
        set(gcf,'units','normalized','outerposition',[0 0 1 1]);
        drawnow;

        if pic_save == true
            Frame=getframe(gcf);
            Image=frame2im(Frame);
            [Image,map]=rgb2ind(Image,256);
            if ii == 1
                writer = VideoWriter('output_video.mp4', 'MPEG-4');
                writer.FrameRate = 30;
                writer.Quality = 100; % 设置视频质量（可选）
                open(writer);
                imwrite(Image,map,output_gif_path,'gif', 'Loopcount',inf,'DelayTime',1/fps);
            else
                writeVideo(writer, Frame);
                imwrite(Image,map,output_gif_path,'gif','WriteMode','append','DelayTime',1/fps);
            end
        end
    end
end
%% Spectrogram
% Interpolation of the data:
% fs = 3000; % new frame rate
TimeSamples = linspace(0,T,Nframes);
NframesNew = round(T*fs); % Number of frame after interpolation
TimeSamplesNew = linspace(0,T,NframesNew);
keypointsNew = zeros(length(TimeSamplesNew),Njoints,3);
for j=1:Njoints
    for k=1:3
        keypointsNew(:,j,k) = interp1(TimeSamples, keypoints(:,j,k),...
            TimeSamplesNew,'spline','extrap');
    end
end
% Calculate Radar returns from target
% Radar parameters
c = 3e8; % m/s
% fc = 60.48e9; % carrier frequency
lambda = c/fc; %(m) wavelength
for nf = 1:NframesNew
    rcs = 0;
    for jj = 1:1:size(connections,1)
        joint1(1:3) = keypointsNew(nf,connections(jj,1),1:3);
        joint2(1:3) = keypointsNew(nf,connections(jj,2),1:3);
        % origin of constructed ellipsoid
        mid = 0.5*(joint1+joint2);
        R_Tx = norm(mid-Tx_pos);
        R_Rx = norm(mid-Rx_pos);
        % aspect vector
        aspect = joint1 - joint2;
        % semi-axis length
        a = norm(aspect)/4;
        b = norm(aspect)/4;
        c = norm(aspect)/2;
        % Calculate theta
        Cos_Theta_i = dot(Tx_pos-mid,aspect)/norm(mid-Tx_pos)/norm(aspect);
        Theta_i = acos(Cos_Theta_i);
        Cos_Theta_s = dot(Rx_pos-mid,aspect)/norm(mid-Rx_pos)/norm(aspect);
        Theta_s = acos(Cos_Theta_s);
        % Calculate phi
        Sin_Phi_i = (Tx_pos(2) - mid(2))/sqrt((Tx_pos(1)-mid(1))^2+(Tx_pos(2)-mid(2))^2);
        Phi_i = asin(Sin_Phi_i);
        Sin_Phi_s = (Rx_pos(2) - mid(2))/sqrt((Rx_pos(1)-mid(1))^2+(Rx_pos(2)-mid(2))^2);
        Phi_s = asin(Sin_Phi_s);
        % rcsellipsoid/R^2 is based on bistatic radar range equation
        Amp = sqrt(rcsellipsoid(a,b,c,Phi_i,Theta_i,Phi_s,Theta_s))/R_Rx/R_Tx;
        Phase = exp(-1i*2*pi*(R_Tx+R_Rx)/lambda);
        rcs_joint = Amp*Phase;
        if isnan(rcs_joint)
            rcs_joint = 0;
        end
        rcs = rcs + rcs_joint;
    end
    RCS(nf) = rcs;
end

RCS = RCS + (AWGN_mean + AWGN_var*rand(size(RCS)));

%% micro-Doppler signature
% 1/fs*v < c/fc; fs>fc/c*v
F = fs;
% thres_A_TRD = -60;
figure;% figure('Position',[500 200 900 600])
colormap(jet)
spectrogram(RCS,kaiser(256,15),250,512,F,'centered','yaxis');
clim = get(gca,'CLim');
% ylim([-0.6 0.6]);
% set(gca,'CLim',clim(2) + [thres_A_TRD 0]);
set(gca,'CLim',clim(2) + [-60 0]);
title('Micro-Doppler Signature', 'Fontsize',12,'color','k')
tightfig;
drawnow

%% Normalize
figure;
F = fs;
[s,f,t] = spectrogram(RCS,kaiser(256,15),250,512,F,'centered','yaxis');
s = s/max(abs(s),[],'all');
s = mag2db(abs(s));
h = imagesc(t, f, s);
xlabel('Time (s)')
ylabel('Doppler frequency (Hz)')
ylim([-600 600]);
axis xy; % 设置坐标轴方向，使频率轴朝上
colormap jet;
colorbar;
caxis([thres_A_TRD,0]);
tightfig;

%% Remove lim
fig = figure;
ax = axes;
new_handle = copyobj(h,ax);
ylim([-600 600]);
xlim([t(1) t(end)]);
colormap jet;
caxis([thres_A_TRD,0]);
axis off
set(gca,'xtick',[],'ytick',[],'xcolor','w','ycolor','w')
set(gca,'looseInset',[0 0 0 0]);
if pic_save == true
    saveas(gcf,output_jpg_path)
end
end