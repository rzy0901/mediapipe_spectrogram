clc; clear; close all;
HAND_PALM_CONNECTIONS = [1 2; 1, 6; 10 14; 14 18; 6 10; 1 18];
HAND_THUMB_CONNECTIONS = [2 3; 3 4; 4 5];
HAND_INDEX_FINGER_CONNECTIONS = [6 7; 7 8; 8 9];
HAND_MIDDLE_FINGER_CONNECTIONS = [10 11; 11 12; 12, 13];
HAND_RING_FINGER_CONNECTIONS = [14 15; 15 16; 16 17];
HAND_PINKY_FINGER_CONNECTIONS = [18 19; 19 20; 20 21];
HAND_FINGER_CONNECTIONS = [HAND_THUMB_CONNECTIONS; HAND_INDEX_FINGER_CONNECTIONS; HAND_MIDDLE_FINGER_CONNECTIONS; HAND_RING_FINGER_CONNECTIONS; HAND_PINKY_FINGER_CONNECTIONS];
connections = [HAND_PALM_CONNECTIONS; HAND_FINGER_CONNECTIONS];


% names = ["push_pull","beckoned","rub_finger"];
names = ["rub_finger";];
for n = 1:1:length(names)
    name = names(n);
%     if name == "rub_finger" || name == "test"
%         connections = [HAND_THUMB_CONNECTIONS;HAND_INDEX_FINGER_CONNECTIONS];
% %                 connections = HAND_FINGER_CONNECTIONS;
%     end
    input_mat_dir = sprintf("./data/keypoints/%s/",name);
    output_jpg_dir = sprintf("./data/all/%s/",name);
    output_gif_dir = sprintf("./data/gif/%s/",name);
    mkdir(input_mat_dir);
    mkdir(output_jpg_dir);
    mkdir(output_gif_dir);



    %%%%%%%%%%%%%%%%%%%%%

%     % Push_pull
%     Tx_pos = [0 -0.1 -1.5]; % XYZ
%     Rx_pos = [0 -0.1 -0.1]; % XYZ
%     AWGN_mean = 0.02;
%     AWGN_var = 0.0001;
%     using_camera_coordinate = true;
 
%     % Beckoned
%     Tx_pos = [0 -0.1 -1.5]; % XYZ
%     Rx_pos = [0 -0.1 -0.1]; % XYZ
%     AWGN_mean = 0.001;
%     AWGN_var = 0.0001;
%     using_camera_coordinate = true;

    % Rub_finger
    Tx_pos = [0.2 -0.05 -1.5]; % XYZ
    Rx_pos = [0.2 -0 0.1]; % XYZ
%     Tx_pos = [0.2 0 0]; % XYZ
%     Rx_pos = [0.2 0 0]; % XYZ
    AWGN_mean = 0.001;
    AWGN_var = 0.0001;
    using_camera_coordinate = true;

    fc = 60.48e9;
    fs = 2000;
    drawScenario = true;
    rcsRendering = true;
    thres_A_TRD = -30;
    pic_save = false;

    %%%%%%%%%%%%%%%%%%%%%



    mat_files = dir(fullfile(input_mat_dir,"*.mat"));
    for ii = 2:1:length(mat_files)
        fprintf("%s: %ds/%d.\n",name,ii,length(mat_files));
        fprintf("Using %s.\n",mat_files(ii).name);
        [~,base,~]= fileparts(mat_files(ii).name);
        input_mat_path = fullfile(mat_files(ii).folder,mat_files(ii).name);
        load(input_mat_path);
        avg_pos = mean(squeeze(keypoints(1,:,:)),1);
        %         avg_pos = [keypoints(1,10,1),keypoints(1,10,2),keypoints(1,10,3)];
        output_jpg_path = fullfile(output_jpg_dir,base+"_s.jpg");
        output_gif_path = fullfile(output_gif_dir,base+"_s.gif");
        simuSpectrogram(Tx_pos,Rx_pos,fc,fs,AWGN_mean,AWGN_var,thres_A_TRD, ...
            drawScenario,rcsRendering, input_mat_path,using_camera_coordinate,connections,output_jpg_path,output_gif_path,pic_save);
        close all;
    end
end