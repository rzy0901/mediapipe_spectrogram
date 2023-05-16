clear; clc; close all;
load('./output/push_pull.mat');
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
        hand = plot3(z,x,y,'.','markersize', 30,'Color',"blue");
        hold on;
        axis equal;
        xmin = min([0 Radar_pos(1) min(keypoints(:,:,1),[],'all')]);
        xmax = max([0 Radar_pos(1) max(keypoints(:,:,1),[],'all')]);
        ymin = min([0 Radar_pos(2) min(keypoints(:,:,2),[],'all')]);
        ymax = max([0 Radar_pos(2) max(keypoints(:,:,2),[],'all')]);
        zmin = min([0 Radar_pos(3) min(keypoints(:,:,3),[],'all')]);
        zmax = max([0 Radar_pos(3) max(keypoints(:,:,3),[],'all')]);
        xlim([zmin zmax]); % Z
        ylim([xmin xmax]); % X
        zlim([ymin ymax]); % Y
%         view(30,30)
%         view(2)
%         view(30,5)
        camera = scatter3(0,0,0,200,"red",'o','DisplayName','Camera');
        radar = scatter3(Radar_pos(3),Radar_pos(1),Radar_pos(2),200,"black",'*','DisplayName','Radar');
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
        set(gcf,'units','normalized','outerposition',[0 0 1 1]);
        set(gca,'ZDir','reverse'); %Y
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
fs = 1500; % new frame rate
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
        R = norm(mid-Radar_pos);
        % aspect vector
        aspect = joint1 - joint2;
        % semi-axis length
        a = norm(aspect)/2;
        b = norm(aspect)/2;
        c = norm(aspect);
        % Calculate theta
        Cos_Theta = dot(Radar_pos-mid,aspect)/norm(mid-Radar_pos)/norm(aspect);
        Theta = acos(Cos_Theta);
        % Calculate phi
        Sin_Phi = (Radar_pos(2) - mid(2))/sqrt((Radar_pos(1)-mid(1))^2+(Radar_pos(2)-mid(2))^2);
        Phi = asin(Sin_Phi);
        % rcsellipsoid/R^2 is based on monostatic radar range equation
        Amp = rcsellipsoid(a,b,c,Phi,Theta)/R^2;
        Phase = exp(-1i*4*pi*R/lambda); 
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