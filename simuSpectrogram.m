function simuSpectrogram(Tx_pos,Rx_pos,fc,fs,AWGN_mean,AWGN_var,thres_A_TRD, ...
    drawScenario,rcsRenderinng,input_mat_path,output_jpg_path,output_gif_path,pic_save)
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
    output_jpg_path (1,1) string
    output_gif_path (1,1) string
    pic_save (1,1) logical
end

load(input_mat_path);
% %%%%%%%%%% Remove first frame %%%%%%%%%%
% keypoints = keypoints(2:end,:,:);
% timestampList = timestampList(2:end);
% handword_keypoints = handword_keypoints(2:end,:,:);
% transformations = transformations(2:end,:,:);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
        hand = plot3(z,x,y,'.','markersize', 13,'Color',"blue");
        hold on;
        axis equal;
        xmin = min([0 Tx_pos(1) Rx_pos(1) min(keypoints(:,:,1),[],'all')]);
        xmax = max([0 Tx_pos(1) Rx_pos(1) max(keypoints(:,:,1),[],'all')]);
        ymin = min([0 Tx_pos(2) Rx_pos(2) min(keypoints(:,:,2),[],'all')]);
        ymax = max([0 Tx_pos(2) Rx_pos(2) max(keypoints(:,:,2),[],'all')]);
        zmin = min([0 Tx_pos(3) Rx_pos(3) min(keypoints(:,:,3),[],'all')]);
        zmax = max([0 Tx_pos(3) Rx_pos(3) max(keypoints(:,:,3),[],'all')]);
        xlim([zmin zmax]); % Z
        ylim([xmin xmax]); % X
        zlim([ymin ymax]); % Y
        %         view(30,30)
        %         view(2)
        %         view(30,5)
        camera = scatter3(0,0,0,100,"red",'o');
        tx = scatter3(Tx_pos(3),Tx_pos(1),Tx_pos(2),100,"magenta",'*');
        rx = scatter3(Rx_pos(3),Rx_pos(1),Rx_pos(2),100,"black",'*');
        for jj = 1:1:size(connections,1)
            plot3(z(connections(jj,:)),x(connections(jj,:)),y(connections(jj,:)),'Color','b','LineWidth',0.05);
        end
        %         for nj=1:Njoints
        %             line([Tx_pos(3) z(nj)],[Tx_pos(1) x(nj)],...
        %                 [Tx_pos(2) y(nj)],'LineStyle',':',...
        %                 'color',[0.5 0.5 0.5],'LineWidth',0.05)
        %             line([Rx_pos(3) z(nj)],[Rx_pos(1) x(nj)],...
        %                 [Rx_pos(2) y(nj)],'LineStyle',':',...
        %                 'color',[0.5 0.5 0.5],'LineWidth',0.05)
        %         end
        for jj = 1:1:size(connections)
            joint1(1:3) = keypoints(ii,connections(jj,1),1:3);
            joint2(1:3) = keypoints(ii,connections(jj,2),1:3);
            mid = 0.5*(joint1+joint2);
            line([Tx_pos(3) mid(3)],[Tx_pos(1) mid(1)],...
                [Tx_pos(2) mid(2)],'LineStyle',':',...
                'color',[0.5 0.5 0.5],'LineWidth',0.05)
            line([Rx_pos(3) mid(3)],[Rx_pos(1) mid(1)],...
                [Rx_pos(2) mid(2)],'LineStyle',':',...
                'color',[0.5 0.5 0.5],'LineWidth',0.05)
            if rcsRenderinng == true
                aspect = joint1 - joint2;
                a = norm(aspect)/4;
                b = norm(aspect)/4;
                c = norm(aspect)/2;
                [X,Y,Z] = ellipsoid2P(joint1,joint2,a,b,c,10);
                surf(Z,X,Y);
            end
        end
        xlabel('Z(m)'); ylabel('X(m)'); zlabel('Y(m)'); title(sprintf('Timestamp: %f (ms)',timestampList(ii)));
        grid on;
        legend([camera tx rx hand] ,'camera','transmitter','receiver','hand joints');
        set(gca,'ZDir','reverse'); %Y
        set(gcf,'units','normalized','outerposition',[0 0 1 1]);
        drawnow;
        Frame=getframe(gcf);
        Image=frame2im(Frame);
        [Image,map]=rgb2ind(Image,256);
        if pic_save == true
            if ii == 1
                imwrite(Image,map,output_gif_path,'gif', 'Loopcount',inf,'DelayTime',1/fps);
            else
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
    for jj = 1:1:size(HAND_FINGER_CONNECTIONS,1)
        joint1(1:3) = keypointsNew(nf,HAND_FINGER_CONNECTIONS(jj,1),1:3);
        joint2(1:3) = keypointsNew(nf,HAND_FINGER_CONNECTIONS(jj,2),1:3);
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
        Amp = rcsellipsoid(a,b,c,Phi_i,Theta_i,Phi_s,Theta_s)/R_Rx/R_Tx;
        Phase = exp(-1i*4*pi*(R_Tx+R_Rx)/lambda);
        rcs_joint = Amp*Phase;
        if isnan(rcs_joint)
            rcs_joint = 0;
        end
        rcs = rcs + rcs_joint;
    end
    RCS(nf) = rcs;
end
RCS = RCS + AWGN_mean + AWGN_var*rand(size(RCS));

%% micro-Doppler signature
% 1/fs*v < c/fc; fs>fc/c*v
F = fs;
% thres_A_TRD = -60;
figure;% figure('Position',[500 200 900 600])
colormap(jet)
spectrogram(RCS,kaiser(256,15),250,512,F,'centered','yaxis');
clim = get(gca,'CLim');
% ylim([-0.6 0.6]);
set(gca,'CLim',clim(2) + [thres_A_TRD 0]);
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