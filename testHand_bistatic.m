clear; clc;
addpath("./utils/");
Tx_pos = [0 -0.1 -1.5]; % XYZ
Rx_pos = [0 -0.1 0.1]; % XYZ
% Tx_pos = [0.4 -0.1 -1]; % XYZ
% Rx_pos = [0.4 -0.1 0.1]; % XYZ
fc = 60.48e9;
fs = 2000;
% AWGN_mean = 0.05;
% AWGN_var = 0.01;
% AWGN_mean = 0.005;
% AWGN_var = 0.0001;
AWGN_mean = 0;
AWGN_var = 0;
thres_A_TRD = -40;
drawScenario = false;
rcsRendering = true;
using_camera_coordinate = true;


HAND_PALM_CONNECTIONS = [1 2; 1, 6; 10 14; 14 18; 6 10; 1 18];
HAND_THUMB_CONNECTIONS = [2 3; 3 4; 4 5];
HAND_INDEX_FINGER_CONNECTIONS = [6 7; 7 8; 8 9];
HAND_MIDDLE_FINGER_CONNECTIONS = [10 11; 11 12; 12, 13];
HAND_RING_FINGER_CONNECTIONS = [14 15; 15 16; 16 17];
HAND_PINKY_FINGER_CONNECTIONS = [18 19; 19 20; 20 21];
HAND_FINGER_CONNECTIONS = [HAND_THUMB_CONNECTIONS; HAND_INDEX_FINGER_CONNECTIONS; HAND_MIDDLE_FINGER_CONNECTIONS; HAND_RING_FINGER_CONNECTIONS; HAND_PINKY_FINGER_CONNECTIONS];
connections = [HAND_PALM_CONNECTIONS; HAND_FINGER_CONNECTIONS];

close all;
name = 'rub_fingers';
input_mat_path = sprintf('./output2/%s.mat',name);
output_jpg_path = sprintf('./output2/%s.jpg',name);
output_gif_path = sprintf('./output2/%s.gif',name);
tic
simuSpectrogram(Tx_pos,Rx_pos,fc,fs,AWGN_mean,AWGN_var,thres_A_TRD, ...
    drawScenario,rcsRendering,input_mat_path,using_camera_coordinate, ...
    connections,output_jpg_path,output_gif_path,false);
toc
