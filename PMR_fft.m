clear all;
% close all;
clc;
f_s = 10e6;
CIT = 0.1;
N_slide = 10;
T_slide = CIT / N_slide;%每0.1s数据移动
sample_per_CIT = CIT*f_s;
max_dop = 1000;
step_dop = 1/CIT;
array_Doppler_frequency = [-max_dop:step_dop:max_dop];
% t_axis = 0:1/f_s:(CIT*f_s-1)/f_s;
%时间刻度
filename = 'push_pull_1.dat';
fullfilename = sprintf('%s/%s','/home/rzy/Documents/data_lc',filename);
picfoldname = '/home/rzy/Documents/data_lc/pic/';
figname = split(filename,'.');
figname = sprintf('%s%s.png',picfoldname,figname{1,1});
pic_save = 0;

%% Read Data File
fid=fopen(fullfilename,'rb');
fseek(fid,0,'eof');
fsize = ftell(fid);
total_samplelen1 = fsize/4;
total_samplelen = total_samplelen1/4;
total_duration = total_samplelen /f_s;
fclose(fid);%关闭文件


fid = fopen(fullfilename,'rb');
fseek(fid,0,'bof');
[data,~]=fread(fid,[2,2*total_samplelen],'float32','l');
data = data(1,:) + 1i * data(2,:);
data = reshape(data,[total_samplelen,2]);
data = data.';
%data 1行是ref， 2行是tar
fclose(fid);
t_axis = 0:1/f_s:(CIT*f_s-1)/f_s;
%%%移动的采样点
S_ref = data(1,1:CIT*f_s);
S_tar = data(2,1:CIT*f_s);
h_temp = xcorr(S_ref,S_tar);
[row_h,col_h] = size(h_temp);
col_h = col_h +1;
N = floor(CIT*f_s)/1000-1;
% N = 10000;
col_max = find(max(abs(h_temp(col_h/2-N:col_h/2+N))) == abs(h_temp(col_h/2-N:col_h/2+N)));
array_sample_shift = col_max - N -1
if array_sample_shift>0
    data_cor(1,:) = data(1,1+array_sample_shift:end);
    data_cor(2,:) = data(2,1:end-array_sample_shift);
else
    data_cor(1,:) = data(1,1:end+array_sample_shift);
    data_cor(2,:) = data(2,1-array_sample_shift:end);
end
clear data;
%%%滑动数据
array_start_time = [0:T_slide:total_duration-CIT];
A_TD = zeros(length(array_start_time),length(array_Doppler_frequency));
%% Process
temp_ref = zeros(1,CIT*f_s);
temp_tar = zeros(1,CIT*f_s);
num_sample = CIT*f_s;
% st = cputime;
tic
for idx_start_time = 1:length(array_start_time)-1
if mod(idx_start_time,50) == 1
    fprintf("idx_start_time: %d / %d\n",idx_start_time,length(array_start_time)-1);
end
temp_ref = data_cor(2,(round(array_start_time(idx_start_time)*f_s+1)):(round(array_start_time(idx_start_time)*f_s)+round(CIT*f_s)));
temp_tar = data_cor(1,(round(array_start_time(idx_start_time)*f_s+1)):(round(array_start_time(idx_start_time)*f_s)+round(CIT*f_s)));
% temp = zeros(1,length(array_Doppler_frequency));
%% fft
temp = fftshift(fft(temp_tar.*conj(temp_ref),num_sample));
A_TD(idx_start_time,:) = temp(num_sample/2+1-max_dop/step_dop:num_sample/2+1+max_dop/step_dop);
end
toc
% fprintf('time cost : %d \n',round(cputime - st));
%% Plot TD figure
thres_A_TRD = -30;
fig1 = figure();
% set(fig1,'position',[50,50,900,600]);
plot_A_DT = abs(A_TD');
plot_A_DT = mag2db(plot_A_DT/max(max(plot_A_DT)));
h = imagesc(array_start_time,array_Doppler_frequency,plot_A_DT);
xlim([array_start_time(1),array_start_time(end)]);
ylim([array_Doppler_frequency(1),array_Doppler_frequency(end)]);
% set(gcf,'unit','centimeters','position',[5 3 30 15]);
set(get(gca,'XLabel'),'FontSize',22);
set(get(gca,'YLabel'),'FontSize',22);
colorbar;
xlabel('Time (s)')
ylabel('Doppler frequency (Hz)')
ylim([-800 800]);
axis xy;
colormap('jet');
caxis([thres_A_TRD,0]);
if pic_save == 1
    f = getframe(gcf);
    imwrite(f.cdata,figname);
end
