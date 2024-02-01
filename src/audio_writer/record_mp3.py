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
import time


class record_mp3():
    ''' Alternate version of audio_visualiser that exports and saves the recordings as mp3 files
    (to the same location), rather than npy files.
    '''
    def __init__(self, topic, file_to_write):
        rospy.init_node('audio_handler', anonymous=True) # avoid duplicate node names with anonymous=True
        rospy.Subscriber(topic, inputMsg, self.audio_callback)
        #self.audio_data = None
        self.pcm = None 
        self.data = AudioSegment.silent(duration=0) # empty audiosegment that will be appended to each callback
        '''
        self.data = np.array([]) # initialise empty np array
        '''
        #self.count = np.array(0) # counter to be used as key for np.savez input
        self.count = 0
        self.file_to_write = file_to_write
        self.lock = Lock() # create threadlock for thread synchronisation
        
        
    def audio_callback(self, data):
        '''
        Callback function converts raw mp3 -> AudioSegment -> appends to instance variable
        
        AudioSegment accepts either file path to mp3 file or raw mp3 data (in the form of bytestrings) as input
        Since data.data accesses the raw audio data as a bytestring, no further processing is needed to convert
        it to a bytestring (np.array(input) and np_array.tobytes() do not have to be used).
        '''
        audio_samples = data.data # need the .data field to access the mp3 data (hexadecimal bytestring of the form '\xff\xf3\xb8\xc4\r}d\x15\xd8')
        
        
        # create AudioSegment object from the raw mp3 data
        audio_segment = AudioSegment(
            audio_samples,  
            sample_width=2, # sample width is number of bytes used to represent one element-> S16LE means 16 bit byte width
            frame_rate=16000,
            channels=1
        )
        
        self.data = self.data + audio_segment # append the new data to self.data
        self.count += 1
        
        
        # Call visualise method for plotting
        #self.visualise_audio()
        
       
    
    def shutdown_callback(self):
        # save audiosegment data as an mp3 file
        self.data.export(self.file_to_write, format="mp3")
        
        # stop timing
        end = time.time()
        print(end - start)