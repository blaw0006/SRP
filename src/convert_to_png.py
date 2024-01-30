#! /usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from pydub import AudioSegment
from audio_common_msgs.msg import AudioData as inputMsg
from io import BytesIO
#from StringIO import StringIO # NOTE: this only works with python2
from io import StringIO # correct import for python3
from threading import Lock
from scipy.signal import butter, lfilter
import math
from PIL import Image
import os
import librosa

def convert_to_png(filename, source, dest, label):
    '''
    Takes as input a mel spectrogram in np array format and converts it to a png
    Inputs
    - label: int representing the label for that audio clip. 0 for no_collision, 1 for collision
    '''
    # load the spectrogram from source
    source = os.path.join(source, filename)
    spectrogram = np.load(source)

    # create the spectrogram image
    fig, ax = plt.subplots()

    S_dB = librosa.power_to_db(spectrogram, ref=np.max)

    img = librosa.display.specshow(S_dB, 
                            #x_axis='time', # exclude labels 
                            #y_axis='mel', 
                            sr=6300, # sampling rate determines time length
                            fmax=200, ax=ax)

    # labels and titles
    #fig.colorbar(img, ax=ax, format='%+2.0f dB')
    #ax.set(title='Mel-frequency spectrogram')

    # uncomment plt.show() to visualise the graph
    #plt.show()

    #return Image.fromarray(np_array, 'RGB')

    
    # save to destination
    f, extension = os.path.splitext(filename) # separates the filename from npy extension
    f_dest = f + ".png" # add the png extension to filename
    f_dest = str(label) + f_dest # add the label to the start of the filename
    dest = os.path.join(dest, f_dest)
    plt.savefig(dest, bbox_inches='tight', format='png')
    plt.close()
    return None
    

if __name__ == '__main__':
    # Create filepaths - edit path as needed depending on device
    path = "/home/caitlin/blaw_ws/src/ur5_control/src/SRP/src" # need / at the start

    source_dir = "spectra" # spectrogram directory
    destination_dir = "pngs" # png storage directory

    collision = "collision"
    no_collision = "no_collision"

    source_collision = os.path.join(path, source_dir, collision)
    source_no_collision = os.path.join(path, source_dir, no_collision)
    png_collision = os.path.join(path, destination_dir, collision)
    png_no_collision = os.path.join(path, destination_dir, no_collision)
    
    # visualise a spectrogram
    # filename = "mic1_test1.npy"
    # convert_to_png(filename, source_collision, png_collision)
    
    # save all collision arrays as pngs to the destination directory
    for filename in os.listdir(source_collision):
        convert_to_png(filename, source_collision, png_collision, 1) # label==1 for collision
        '''
        spectrogram = np.load(os.path.join(source_collision, filename))
        #print(spectrogram)
        #print(spectrogram.shape)
        im = convert_to_png(spectrogram)
        f, extension = os.path.splitext(filename) # separates the filename from npy extension
        f_dest = f + ".png" # add the png extension 
        #im.save(os.path.join(png_collision, f_dest))

        #print(filename)
        #print(os.path.join(png_collision, f_dest))
        '''

    count = 0
    for filename in os.listdir(source_no_collision):
        print(count)
        count += 1
        convert_to_png(filename, source_no_collision, png_no_collision, 0) # label==0 for no collision
        '''
        spectrogram = np.load(os.path.join(source_no_collision, filename))
        im = convert_to_png(spectrogram)
        f, extension = os.path.splitext(filename) # separates the filename from npy extension
        f_dest = f + ".png" # add the png extension 
        #im.save(os.path.join(png_no_collision, f_dest))
        '''
    
