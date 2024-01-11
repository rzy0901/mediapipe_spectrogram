import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
from mpl_toolkits.mplot3d import Axes3D



### Read files
# foldname = 'I:\Passive_mmwave\Datasets\PMR Walk\data_0607'
# filename ='\go_back1.dat'
# fname = foldname + filename
# data = np.fromfile(fname,dtype = '<f', count = -1,).reshape(2,-1,order = "F")

def PMR_fft(data):
    fs = 10e6
    CIT = 0.1
    N_slide = 10
    T_slide = CIT / N_slide
    #数据转化为两行
    print(data.shape,type(data))
    # data_complex = data[0,:] + 1j*data[1,:]
    # data_sample = data.reshape(-1,2,order = "F")
    data_sample = data
    # print(data_sample.shape)
    # data_sample = np.transpose(data_sample)
    #第一行是tar，第二行是ref
    num_sample = round(CIT * fs)
    data_tar = data_sample[1,:num_sample]
    data_ref = data_sample[0,:num_sample]
    plt.figure(1)
    plt.subplot(211)
    plt.plot(data_tar[:2184*2].real)
    plt.title('sur signal')
    plt.subplot(212)
    plt.plot(data_ref[:2184*2].real)
    plt.title('ref signal')
    print(data_ref.shape)
    # print('data shape:',data_tar.shape,'data type:',type(data_tar[1,1]))z

    total_duration = round(len(data_sample[1,:]) / fs)

    N = 10000
    h_corr = np.correlate(data_ref[:N*2+1],data_tar[:N *2+1],'same')
    h_corr = abs(h_corr)
    h_corr_max = np.argmax(h_corr) 

    ##process
    array_sample_shift = h_corr_max - N 
    array_start_time =np.arange(0,total_duration-CIT,T_slide)
    Doppler_frequency = 600
    step_dop = 1/CIT
    # t_axis = np.arange(0,(CIT*fs-1)/fs,1/fs)
    array_Doppler_frequency = np.arange(-Doppler_frequency,Doppler_frequency,step_dop)

    if  array_sample_shift > 0:
        data_ref_cor = data_sample[0,array_sample_shift:]
        data_tar_cor = data_sample[1,:-array_sample_shift]
    else:
        data_ref_cor = data_sample[0,:array_sample_shift]
        data_tar_cor = data_sample[1,-array_sample_shift:]
    del data_ref,data_tar,data_sample

    num_loop = len(array_Doppler_frequency)
    A_TD = np.zeros((len(array_start_time),num_loop),dtype = 'complex_')
    for idx in tqdm(range(len(array_start_time))):
        temp_tar = data_tar_cor[round(array_start_time[idx]*fs):round(array_start_time[idx]*fs) + num_sample]
        temp_ref = data_ref_cor[round(array_start_time[idx]*fs):round(array_start_time[idx]*fs) + num_sample]
        A_TD[idx,:] =  np.fft.fftshift(np.fft.fft(temp_tar*np.conj(temp_ref),num_sample))[round(num_sample/2+1-Doppler_frequency/step_dop):round(num_sample/2+1+Doppler_frequency/step_dop)]
    ###plot figure
    # np.savetxt("A_TRD.txt",A_TRD)
    x, y = np.meshgrid(array_start_time,array_Doppler_frequency)
    plot_A_TD = np.zeros(np.size(A_TD))
    plot_A_TD = 20*(np.log(abs(A_TD)/np.max((np.max(abs(A_TD)))))/np.log(20))
    plt.figure(2)
    # plt.pcolor(x,y,plot_A_TD.T,cmap='jet',vmin=-20,vmax=-10)
    plt.pcolor(x,y,plot_A_TD.T,cmap='jet',vmin=-20,vmax=-10)
    plt.colorbar
    
    
    plt.show()
   

def data_save(data):
    if_save = input('if save : 1\n')

    if if_save == '1':
        name = input('input file name:\n')
        ################################
        filename = '/home/lab525pr/mmwave_PR/Dataset/data_lc/'+name+'.dat'
        with open(filename, 'wb') as fid:
            data.tofile(fid)
        print("[succ] File saving complete.\n")
