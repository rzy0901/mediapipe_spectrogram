import numpy as np
from utils.PMR_fft import PMR_fft

## Read files
foldname = '/home/rzy/Documents/data_lc/'
filename ='push_pull_1.dat'
fname = foldname + filename
data = np.fromfile(fname,dtype = '<f', count = -1,).reshape(2,-1,order = "F")
data_complex = data[0,:] + 1j*data[1,:]
data_sample = data_complex.reshape(-1,2,order = "F")
data_sample = np.transpose(data_sample)
PMR_fft(data_sample)