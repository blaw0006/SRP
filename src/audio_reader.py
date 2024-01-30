#! /usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from pydub import AudioSegment
from audio_common_msgs.msg import AudioData as inputMsg
from io import BytesIO
from StringIO import StringIO # NOTE: this only works with python2
#from io import StringIO # correct import for python3
from threading import Lock
from scipy.signal import butter, lfilter
import math

def butter_bandpass(lowcut, highcut, fs, order=5):
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


def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
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
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y

def audio_reader(file, file2, save):
    '''
    Change file input to list of filenames to plot, plot in loop. May still have issue if matplotlib pauses
    execution while a plot is still running. 
    Inputs
    - file: filename from which to read 
    - save: bool that determines whether the filtered files should be saved in a folder

    Outputs
    - time: numpy time array (length depends on size of data)
    - data: numpy array representing the edited soundwave data (over time)
    '''
    # Load from file
    data = np.load(file)
    
    
    # convert to decibels: db = 20*log10(amplitude/reference_amplitude=32767)
    data = np.divide(data, 32767)    
    
    # offset to avoid log(0) errors
    data = np.add(data, 1e-10)
    
    data = np.absolute(data)
        
    data = np.log10(data)
    data = np.multiply(data, 20)
    
    #data = np.multiply(20, np.log10(np.divide(data, 32767))) #20*math.log10(data/32767)
    
    print(len(data))
    time = np.arange(0,len(data)).astype(float)/6300 # size of pcm np_array is variable, specify float or it rounds to int
    time_array = np.array([])
    time_array = np.append(time_array, time)
    
    
    # Apply bandpass filter
    lowcut = 20 # <10Hz is on the floor sounds, 20 seems to be the best 
    highcut = 190 # >200Hz is over the air sounds. 190 seems to be the lowest it can go without filtering everything
    sample_rate = 6300 # approximate, may need to change
    
    data = butter_bandpass_filter(data, lowcut, highcut, sample_rate, order = 5)
    
    #print(time_array)
    #print(np.shape(time_array))
    time = np.arange(0,len(data)).astype(float)/6300
    
    # Save to folder if save == 1
    if save:
        np.save(file2, data) # save data to specified filepath

    return time, data


def plot(time, data):
    # Plotting and titles
    figure, axis = plt.subplots()
    #axis.plot(time_array, data)
    axis.plot(time, data)
    axis.set_title('Audio Waveform')
    axis.set_xlabel('Time (seconds)')
    axis.set_ylabel('Decibels')
    
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
    #audio_reader("src/ur5_control/src/bruh.npy")
    
    # change i range to match the number of tests
    test_start = input("Enter test start: ")x
    test_end = input("Enter test end: ")
    
    for i in range(test_start,test_end+1):
        filename = "src/ur5_control/src/two_mic_tests/mic1_test" + str(i) + ".npy"
        time, data = audio_reader(filename, "NA", 0)
        plot(time, data)
        
        filename = "src/ur5_control/src/two_mic_tests/mic2_test" + str(i) + ".npy"
        time, data = audio_reader(filename, "NA", 0)
        plot(time, data)
    
    #audio_reader("src/ur5_control/src/two_mic_tests/mic1_test1.npy")
    #audio_reader("src/ur5_control/src/two_mic_tests/mic2_test1.npy")
    
    