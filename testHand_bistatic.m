clear; clc; 
% close all;
load('./output/push_pull_1.mat');
Njoints = size(keypoints,2);
Nframes = length(timestampList);
frameLength = 1/28.5; % fps =  28.5;
HAND_PALM_CONNECTIONS = [1 2; 1, 6; 10 14; 14 18; 6 10; 1 18];
HAND_THUMB_CONNECTIONS = [2 3; 3 4; 4 5];
HAND_INDEX_FINGER_CONNECTIONS = [6 7; 7 8; 8 9];
HAND_MIDDLE_FINGER_CONNECTIONS = [10 11; 11 12; 12, 13];
HAND_RING_FINGER_CONNECTIONS = [14 15; 15 16; 16 17];
HAND_PINKY_FINGER_CONNECTIONS = [18 19; 19 20; 20 21];
connection = [HAND_PALM_CONNECTIONS; HAND_THUMB_CONNECTIONS; HAND_INDEX_FINGER_CONNECTIONS; HAND_MIDDLE_FINGER_CONNECTIONS; HAND_RING_FINGER_CONNECTIONS; HAND_PINKY_FINGER_CONNECTIONS];
T = frameLength*Nframes;
Tx_pos = [0 -0.1 -1.5]; % XYZ
Rx_pos = [0 -0.1 0]; % XYZ
drawScenario = true;
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
        for jj = 1:1:size(connection,1)
            plot3(z(connection(jj,:)),x(connection(jj,:)),y(connection(jj,:)),'Color','b','LineWidth',0.05);
        end
        for nj=1:Njoints        
            line([Tx_pos(3) z(nj)],[Tx_pos(1) x(nj)],...
                           [Tx_pos(2) y(nj)],'LineStyle',':',...
                           'color',[0.5 0.5 0.5],'LineWidth',0.05)
            line([Rx_pos(3) z(nj)],[Rx_pos(1) x(nj)],...
                           [Rx_pos(2) y(nj)],'LineStyle',':',...
                           'color',[0.5 0.5 0.5],'LineWidth',0.05)
        end    
        xlabel('Z(m)'); ylabel('X(m)'); zlabel('Y(m)'); title(sprintf('Timestamp: %f (ms)',timestampList(ii)));
        grid on;
        legend([camera tx rx hand] ,'camera','transmitter','receiver','hand');
        set(gca,'ZDir','reverse'); %Y
%         set(gcf,'units','normalized','outerposition',[0 0 1 1]);
        drawnow;
        Frame=getframe(gcf);
        Image=frame2im(Frame);
        [Image,map]=rgb2ind(Image,256);
        if ii == 1
            imwrite(Image,map,'test.gif','gif', 'Loopcount',inf,'DelayTime',0.033);
        else
            imwrite(Image,map,'test.gif','gif','WriteMode','append','DelayTime',0.033);
        end
    end
end
%% Spectrogram
% Interpolation of the data:
fs = 3000; % new frame rate
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
fc = 60.48e9; % carrier frequency
lambda = c/fc; %(m) wavelength
for nf = 1:NframesNew
    rcs = 0;
    for jj = 1:1:size(connection,1)
        joint1(1:3) = keypointsNew(nf,connection(jj,1),1:3);
        joint2(1:3) = keypointsNew(nf,connection(jj,2),1:3);
        % origin of constructed ellipsoid
        mid = 0.5*(joint1+joint2);
        R_Tx = norm(mid-Tx_pos);
        R_Rx = norm(mid-Rx_pos);
        % aspect vector
        aspect = joint1 - joint2;
        % semi-axis length
        a = norm(aspect)/2;
        b = norm(aspect)/2;
        c = norm(aspect);
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
        % rcsellipsoid/R^2 is based on monostatic radar range equation
        Amp = rcsellipsoid(a,b,c,Phi_i,Theta_i,Phi_s,Theta_s)/R_Rx/R_Tx;
        Phase = exp(-1i*4*pi*(R_Tx+R_Rx)/lambda); 
        rcs_joint = Amp*Phase;
        rcs = rcs + rcs_joint;
    end
    RCS(nf) = rcs;
end
RCS = RCS + 0.02+0.015*randn(size(RCS));

% % micro-Doppler signature
% % 1/fs*v < c/fc; fs>fc/c*v
% F = fs;
% figure;% figure('Position',[500 200 900 600])
% colormap(jet)
% spectrogram(RCS,kaiser(256,15),250,512,F,'centered','yaxis');
% clim = get(gca,'CLim');
% set(gca,'CLim',clim(2) + [-60 0]);
% title('Micro-Doppler Signature', 'Fontsize',12,'color','k')
% drawnow

figure;
F = fs;
thres_A_TRD = -30;
[s,f,t] = spectrogram(RCS,kaiser(256,15),250,512,F,'centered','yaxis');
s = s/max(abs(s),[],'all');
s = mag2db(abs(s));
imagesc(t, f, s);
xlabel('Time (s)')
ylabel('Doppler frequency (Hz)')
ylim([-800 800]);
axis xy; % 设置坐标轴方向，使频率轴朝上
colormap jet; 
colorbar;
caxis([thres_A_TRD,0]);




