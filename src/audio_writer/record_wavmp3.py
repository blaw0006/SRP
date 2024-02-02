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

class record_wavmp3():
    ''' Alternate version of audio_visualiser that accumulates data as audiosegments then uses the 
    actual pydub conversion function to convert to wav + frombuffer to convert to np, rather than
    using hidden methods to convert to wav as in audio_visualiser
    '''
    def __init__(self, topic, wav_file_to_write, mp3_file_to_write):
        rospy.init_node('record_wav', anonymous=True) # avoid duplicate node names with anonymous=True
        rospy.Subscriber(topic, inputMsg, self.audio_callback)
        self.data = AudioSegment.silent(duration=0) # empty audiosegment that will be appended to each callback
        self.wav_file_to_write = wav_file_to_write
        self.mp3_file_to_write = mp3_file_to_write
        self.lock = Lock() # create threadlock for thread synchronisation
        
        
    def audio_callback(self, data):
        '''
        Callback function converts raw mp3 -> AudioSegment -> wav -> np_array
        
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
        
       
    
    def shutdown_callback(self):
        '''
        Note that all AudioSegment.from_file methods return an AudioSegment object.
        '''
        
        # convert audiosegment to wav/mp3 and export
        self.data.export(self.wav_file_to_write, format="wav") # save wav file first
        self.data.export(self.mp3_file_to_write, format="mp3") # save mp3 file

        # stop timing
        # end = time.time()
        # print(end - start)
