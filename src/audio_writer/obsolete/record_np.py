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


class record_np:
    ''' 
    Records audio as a numpy file, ready to be processed
    Takes individual mp3 frames -> AudioSegment -> wav -> np array and appends np_array to stored instance variable self.data.
    Shutdown function saves full np array to self.file_to_write
    '''
    def __init__(self, topic, file_to_write):
        rospy.init_node('record_np', anonymous=True) # avoid duplicate node names with anonymous=True
        rospy.Subscriber(topic, inputMsg, self.audio_callback)
        self.pcm = None 
        self.data = np.array([]) # initialise empty np array
        self.count = 0
        self.file_to_write = file_to_write
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
        
        # boost the signal by 30db
        #audio_segment = audio_segment + 30
        
        # hidden instance variable containing raw wav data after conversion 
        wav_data = audio_segment._data
        
        # convert wav to np_array for plotting
        self.pcm = np.frombuffer(wav_data, dtype=np.int16) 
        #print(self.pcm)
        
        self.data = np.append(self.data, self.pcm)
        self.count += 1
        
        
        # Call visualise method for plotting
        #self.visualise_audio()
        
       
    
    def shutdown_callback(self):
        print(self.data)
        print(np.shape(self.data))
        
        # save to np file for access later
        np.save(self.file_to_write, self.data)
        
        # stop timing
        end = time.time()
        print(end - start)

