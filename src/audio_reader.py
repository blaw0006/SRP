#! /usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from pydub import AudioSegment
from audio_common_msgs.msg import AudioData as inputMsg
from io import BytesIO
from StringIO import StringIO
from threading import Lock
from scipy.signal import butter, lfilter

def audio_reader(file):
    # exit condition: 
    # load data from file
    
    data = np.load(file)
    
    # plot
    figure, axis = plt.subplots()
    
    print(len(data))
    time = np.arange(0,len(data)).astype(float)/6300 # size of pcm np_array is variable 
    time_array = np.array([])
    time_array = np.append(time_array, time)
    
    print(time_array)
    print(np.shape(time_array))
    
    # Plotting and titles
    #axis.plot(time_array, data)
    axis.plot(time, data)
    axis.set_title('Audio Waveform')
    axis.set_xlabel('Time (seconds)')
    axis.set_ylabel('Amplitude')
    plt.show()
    #plt.pause(0.001)
                    
    
    '''
    data = np.load(file) # npzfile object, requires keys to access individual arrays 
    keys = data.files
    array = None
    
    for key in keys:
        array = data[key]
        print(array)
    '''
    
    def butter_bandpass(self, lowcut, highcut, fs, order=5):
        ''' Helper function that computes filter coefficients for caller 
        From: https://scipy-cookbook.readthedocs.io/items/ButterworthBandpass.html
        Inputs
            lowcut: lower bound for frequency for high pass filter
            highcut: upper bound for frequency for low pass filter
            fs: sampling frequency
        Outputs
            a, b: filter coefficients used in the digital filter later 
        '''
        nyq = 0.5 * fs
        low = lowcut / nyq
        high = highcut / nyq
        b, a = butter(order, [low, high], btype='band')
        return b, a


    def butter_bandpass_filter(self, data, lowcut, highcut, fs, order=5):
        ''' Function that applies a butter bandpass filter to input data
        From: https://scipy-cookbook.readthedocs.io/items/ButterworthBandpass.html
        
        Inputs
            data: data to be filtered
            lowcut: lower bound for frequency for high pass filter
            highcut: upper bound for frequency for low pass filter
            fs: sampling frequency
            order: sharpness of cutoff, see https://stackoverflow.com/questions/12093594/how-to-implement-band-pass-butterworth-filter-with-scipy-signal-butter
        Outputs
            y: the filtered data 
        
        '''
        b, a = self.butter_bandpass(lowcut, highcut, fs, order=order)
        y = lfilter(b, a, data)
        return y
    
if __name__ == '__main__':
    '''
    x = np.arange(10)
    y = np.array(0)
    np.savez("your_file.npz", x=x, y=y)

    # Save the second array to the same file in append mode
    z = np.array(1)
    with np.load("your_file.npz", allow_pickle=True) as data:
        np.savez("your_file.npz", data, z=z)

    # Load the file and access the arrays
    loaded_data = np.load("your_file.npz")
    y_loaded = loaded_data[y]
    print(y_loaded)
    z_loaded = loaded_data['z']
    print(z_loaded)
    
    #audio_reader("bruh.npz")
    x = np.arange(10)
    y = np.array(0)
    z = np.array(1)
    
    
    for i in range(0,10):
        np.savez("src/ur5_control/src/bruh.npz", x, y)
        y += 1
    
    
    np.savez("src/ur5_control/src/bruh.npz", x, y)
    np.savez("src/ur5_control/src/bruh.npz", x, z)
    
    data = np.load("src/ur5_control/src/bruh.npz")
    print(data[y])
    print(data[z])
    '''
    audio_reader("src/ur5_control/src/bruh.npy")
    