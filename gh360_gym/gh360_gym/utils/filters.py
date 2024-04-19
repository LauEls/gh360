from scipy import signal
import numpy as np

def freq_filter(data, f_size, cutoff):
    lgth=data.shape[0]
    f_data=np.zeros(lgth)
    lpf=signal.firwin(f_size, cutoff, window='hamming')
    f_data=signal.convolve(data, lpf, mode='same')
    return f_data
    
def median_filter(data, f_size):
    lgth=data.shape[0]
    f_data=np.zeros(lgth)
    f_data=signal.medfilt(data, f_size)
    return f_data