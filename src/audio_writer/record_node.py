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
import os
import sys

class record_node():
    ''' Alternate version of audio_visualiser that accumulates data as audiosegments then uses the 
    actual pydub conversion function to convert to wav + frombuffer to convert to np, rather than
    using hidden methods to convert to wav as in audio_visualiser
    '''
    def __init__(self, topic, test_number, mic):
        # Initialise important values 
        label = "/collision" # collision clips - affects the path location
        start = "/1" # heading for the clips
        
        # important, used to create or overwrite file for recorded sound
        #test = str(input("Enter test number: "))
        
        # creates a sound file 
        self.wav_file_to_write = "/home/acrv/blaw_ws/src/wav_data" + label + start + "mic" + str(mic) +"_test" + str(test_number) + ".wav"
        self.mp3_file_to_write = "/home/acrv/blaw_ws/src/mp3_data" + label + start + "mic" + str(mic) + "_test" + str(test_number) + ".mp3"
        
        # Check if file exists - if so, terminate and throw error message
        if os.path.exists(self.wav_file_to_write) or os.path.exists(self.mp3_file_to_write): 
            rospy.signal_shutdown("File already exists. Delete it first if you wish to overwrite.")
        
        # if os.path.exists(self.wav_file_to_write) or os.path.exists(self.mp3_file_to_write): 
        #     override = int(input("Test already exists. Enter 1 to overwrite: "))
        #     if override != 1:
        #         sys.exit()
        
        # start timing
        self.start = time.time()
        
        #rospy.init_node('record_wav', anonymous=True) # avoid duplicate node names with anonymous=True
        self.data = AudioSegment.silent(duration=0) # empty audiosegment that will be appended to each callback
        rospy.Subscriber(topic, inputMsg, self.audio_callback)
        rospy.on_shutdown(self.shutdown_callback)
        
    def audio_callback(self, data):
        '''
        Callback function converts raw mp3 -> AudioSegment -> wav -> np_array
        
        AudioSegment accepts either file path to mp3 file or raw mp3 data (in the form of bytestrings) as input
        Since data.data accesses the raw audio data as a bytestring, no further processing is needed to convert
        it to a bytestring (np.array(input) and np_array.tobytes() do not have to be used).
        '''
        audio_samples = data.data # need the .data field to access the mp3 data (hexadecimal bytestring of the form '\xff\xf3\xb8\xc4\r}d\x15\xd8')
        #print(data)
        
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
        # Mic 1
        print(self.mp3_file_to_write)
        
        try:
            self.data.export(self.wav_file_to_write, format="wav") # save wav file first
            self.data.export(self.mp3_file_to_write, format="mp3") # save mp3 file
        except Exception as e:
            rospy.logerr("Error exporting audio data: %s", str(e))

        # stop timing
        end = time.time()
        rospy.loginfo("Recording duration: %.2f seconds", end - self.start)
        
if __name__ == '__main__': 
    rospy.init_node('record_node', anonymous=True)  # Initialize ROS node
    
    # Get params from ros param server (passed there by launch file)
    topic = rospy.get_param("~topic")  
    test_number = rospy.get_param("~test_number")  
    mic = rospy.get_param("~mic")  
    
    # Instantiate record_node and spin
    node = record_node(topic, test_number, mic) 
    rospy.spin()  # Start ROS node
