clear; clc;
Tx_pos = [0 -0.1 -1.5]; % XYZ
Rx_pos = [0 -0.1 0]; % XYZ
fc = 60.48e9;
fs = 3000;
% AWGN_mean = 0.005;
% AWGN_var = 0.001;
AWGN_mean = 0.001;
AWGN_var = 0.0001;
thres_A_TRD = -30;
drawScenario = true;
rcsRendering = true;

% close all;
name = 'push_pull_1';
input_mat_path = sprintf('./output/%s.mat',name);
output_jpg_path = sprintf('./output/%s.jpg',name);
output_gif_path = sprintf('./output/%s.gif',name);
simuSpectrogram(Tx_pos,Rx_pos,fc,fs,AWGN_mean,AWGN_var,thres_A_TRD,drawScenario,rcsRendering,input_mat_path,output_jpg_path,output_gif_path,false);

% pic_save = true;
% files  = dir(fullfile('./output','*.mat'));
% for ii = 1:1:length(files)
%     [~,name,~]= fileparts(files(ii).name);
%     input_mat_path = sprintf('./output/%s.mat',name);
%     output_jpg_path = sprintf('./output/%s.jpg',name);
%     output_gif_path = sprintf('./output/%s.gif',name);
%     simuSpectrogram(Tx_pos,Rx_pos,fc,fs,AWGN_mean,AWGN_var,thres_A_TRD,drawScenario,rcsRendering,input_mat_path,output_jpg_path,output_gif_path,pic_save);
%     close all;
% end

