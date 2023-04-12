clear; clc; close all;
load('./data.mat')
keypoints = keypoints(3:end,:,:);
timestampList = timestampList(3:end);
% keypoints_test = smoothdata(keypoints,1,"rlowess",5);
% % figure;
% % subplot(131),plot(keypoints(:,1,1)); hold on; plot(keypoints_test(:,1,1)); xlabel('frame'); ylabel('x'); legend('raw','smoothed');
% % subplot(132),plot(keypoints(:,1,2)); hold on; plot(keypoints_test(:,1,2)); xlabel('frame'); ylabel('y'); legend('raw','smoothed');
% % subplot(133),plot(keypoints(:,1,3)); hold on; plot(keypoints_test(:,1,3)); xlabel('frame'); ylabel('z'); legend('raw','smoothed');
% keypoints = keypoints_test;
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
Radar_pos = [0 0 0]; % XYZ
drawScenario = false;
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
        hand = plot3(z,x,y,'.','markersize', 13);
        hold on;
        axis equal;
%         xlim([-1 0.2]); % Z
%         ylim([-0.3 0.3]); % X
%         zlim([-0.3 0.3]); % Y
%         view(30,30)
%         view(2)
%         view(30,5)
        camera = scatter3(0,0,0,[],"red",'o','DisplayName','Camera');
        radar = scatter3(Radar_pos(3),Radar_pos(1),Radar_pos(2),[],"black",'*','DisplayName','Radar');
        for jj = 1:1:size(connection,1)
            plot3(z(connection(jj,:)),x(connection(jj,:)),y(connection(jj,:)),'Color','b','LineWidth',0.05);
        end
        for nj=1:Njoints        
            line([Radar_pos(3) z(nj)],[Radar_pos(1) x(nj)],...
                           [Radar_pos(2) y(nj)],'LineStyle',':',...
                           'color',[0.5 0.5 0.5],'LineWidth',0.05)
        end    
        xlabel('Z(m)'); ylabel('X(m)'); zlabel('Y(m)'); title(sprintf('Timestamp: %f (ms)',timestampList(ii)));
        grid on;
        legend([camera radar hand] ,'camera','radar','hand');
        drawnow;
        Frame=getframe(gcf);
        Image=frame2im(Frame);
        [Image,map]=rgb2ind(Image,256);
        if ii == 1
            imwrite(Image,map,'test.gif','gif', 'Loopcount',inf,'DelayTime',0.3);
        else
            imwrite(Image,map,'test.gif','gif','WriteMode','append','DelayTime',0.3);
        end
    end
end
%% Spectrogram
% Interpolation of the data:
fs = 4000; % new frame rate
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
        r1(1:3) = keypointsNew(nf,connection(jj,1),1:3);
        r2(1:3) = keypointsNew(nf,connection(jj,2),1:3);
        r1r2_mid=0.5*(r1+r2);
        R = norm(r1r2_mid-Radar_pos);
        R1 = Radar_pos - r1r2_mid;
        R2 = r2-r1;
        Cos_Theta = ((R1(1)*R2(1))+(R1(2)*R2(2))+(R1(3)*R2(3)))/norm(R1)/norm(R2);
        Theta = acos(Cos_Theta);
        Phase = exp(-1i*4*pi*R/lambda)/R^2; 
        height = norm(r2-r1);
        radius = height/4;
        Amp = sqrt(1/4*pi*radius^4*height^2/(radius^2*(sin(Theta))^2+1/4*height^2*(cos(Theta))^2));
        rcs_joint = Amp*Phase;
        rcs = rcs + rcs_joint;
    end
    RCS(nf) = rcs;
end

% micro-Doppler signature
% 1/fs*v < c/fc; fs>fc/c*v
F = fs;
figure;% figure('Position',[500 200 900 600])
colormap(jet)
spectrogram(RCS,kaiser(256,15),250,512,F,'centered','yaxis');
clim = get(gca,'CLim');
set(gca,'CLim',clim(2) + [-60 0]);
title('Micro-Doppler Signature', 'Fontsize',12,'color','k')
drawnow
