clc; clear; close all;
% input_mat_path = "./data/keypoints/push_pull/1.mat";
% output_jpg_path = "./data/all/push_pull/1.jpg";
% output_gif_path = "./data/gif/push_pull/1.gif";
% simuSpectrogram(input_mat_path,output_jpg_path,output_gif_path)

names = ["push_pull","beckoned","rub_finger"];
% names = ["beckoned"];
for n = 1:1:length(names)
    name = names(n);
    input_mat_dir = sprintf("./data/keypoints/%s/",name);
    output_jpg_dir = sprintf("./data/all/%s/",name);
    output_gif_dir = sprintf("./data/gif/%s/",name);
    mkdir(input_mat_dir);
    mkdir(output_jpg_dir);
    mkdir(output_gif_dir);
    %%%%%%%%%%%%%%%%%%%%%
    Tx_pos = [0 -0.1 -1.5]; % XYZ
    Rx_pos = [0 -0.1 0]; % XYZ
    fc = 60.48e9;
    fs = 5000;
    drawScenario = false;
    rcsRendering = false;
    AWGN_mean = 0.001;
    AWGN_var = 0.0001;
%     AWGN_mean = 0.005;
%     AWGN_var = 0;
    thres_A_TRD = -30;
    pic_save = true;
    %%%%%%%%%%%%%%%%%%%%%
    mat_files = dir(fullfile(input_mat_dir,"*.mat"));
    for ii = 1:1:length(mat_files)
        fprintf("%s: %d/%d.\n",name,ii,length(mat_files));
        [~,base,~]= fileparts(mat_files(ii).name);
        input_mat_path = fullfile(mat_files(ii).folder,mat_files(ii).name);
        output_jpg_path = fullfile(output_jpg_dir,base+".jpg");
        output_gif_path = fullfile(output_gif_dir,base+".gif");
        simuSpectrogram(Tx_pos,Rx_pos,fc,fs,AWGN_mean,AWGN_var,thres_A_TRD, ...
            drawScen ...
            ario,rcsRendering, input_mat_path,output_jpg_path,output_gif_path,pic_save);
        close all;
    end
end