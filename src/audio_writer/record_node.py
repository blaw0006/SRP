#! /usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
#import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from pydub import AudioSegment
from audio_common_msgs.msg import AudioData as inputMsg
from io import BytesIO
#from StringIO import StringIO
from io import StringIO
from threading import Lock
import time
import os
import sys
import re

class record_node():
    ''' Alternate version of audio_visualiser that accumulates data as audiosegments then uses the 
    actual pydub conversion function to convert to wav + frombuffer to convert to np, rather than
    using hidden methods to convert to wav as in audio_visualiser
    '''
    def __init__(self, topic, test_number, mic):
        # Initialise important values 
        collision_label = "/no_collision" # collision clips - affects the path location
        start = "/0" # heading for the clips
        
        # Find test number 
        folder_path = "/home/acrv/blaw_ws/src/4_mic_data/mp3_data" + collision_label

        # Get all files in the folder
        files = os.listdir(folder_path)

        # Regular expression pattern to match test numbers
        pattern = r"test(\d+)_"

        # Initialize the highest test number
        highest_test_num = 0

        # Iterate through the files and extract test numbers
        if not files:
            highest_test_num = 0
        else:
            for file in files:
                match = re.search(pattern, file)
                if match:
                    test_num = int(match.group(1))
                    if test_num > highest_test_num:
                        highest_test_num = test_num

        # Increment the highest test number by 1
        test_number = highest_test_num + 1
        
        # Encode drop position (x,y,z)
        label_num = 0
        
        if label_num == test_number % 68:
            label_num = 67
        else:
            label_num = test_number % 68 - 1 # -1 due to zero indexing
            
        #tens = (test_number // 10) % 10
        #ones = test_number % 10
        #label_num = tens * 10 + ones
        z = 10
        
        if 17<=label_num<=33:
            label_num -= 17
            z = 20
        elif 34<=label_num<=50:
            label_num -= 34
            z = 30
        elif 51<=label_num<=67:
            label_num -= 51
            z = 40
        
        # obtain label
        labels = ["x=-60,y=20,",
            "x=0,y=20,",
            "x=60,y=20,",
            "x=0,y=10,",
            "x=-16.66,y=5,",
            "x=16.66,y=5,",
            "x=-50,y=0,",
            "x=-33.33,y=0,",
            "x=0,y=0,",
            "x=33.33,y=0,",
            "x=50,y=0,",
            "x=-16.66,y=-5,",
            "x=16.66,y=-5,",
            "x=0,y=-10,",
            "x=-60,y=-20,",
            "x=0,y=-20,",
            "x=60,y=-20,",
        ]
        
        label = labels[label_num]
        label = "_position(" + label + "z=" + str(z) + ")_"
        
        # print messages
        print("Drop position: %d" % (label_num + 1))
        if test_number % 17 == 0:
            print("########\nLast drop for this height, switch to height %s for next drop" % (z+10))
        if test_number % 68 == 0:
            print("########\nLast drop for this object, switch object for next drop.")
        
        
        # Encode mic position
        mic_positions = ["(x=50,y=10)", "(x=-50,y=10)", "(x=-50,y=-10)", "(x=50,y=-10)"]
        mic_position = mic_positions[mic-1]
        
        # Encode object
        object_num = (test_number) % 612
        if object_num % 68 == 0:
            object_num = object_num // 68
        else:
            object_num = object_num // 68 + 1
        object_str = "_object" + str(object_num)
        
        
        
        # important, used to create or overwrite file for recorded sound
        #test = str(input("Enter test number: "))
        
        # creates a sound file 
        #self.wav_file_to_write = "/home/acrv/blaw_ws/src/wav_data" + label + start + "mic" + str(mic) +"_test" + str(test_number) + ".wav"
        #self.mp3_file_to_write = "/home/acrv/blaw_ws/src/mp3_data" + label + start + "mic" + str(mic) + "_test" + str(test_number) + ".mp3"
        self.wav_file_to_write = "/home/acrv/blaw_ws/src/4_mic_data/wav_data" + collision_label + start + "_test" + str(test_number) + label + "mic" + str(mic) + mic_position + object_str + ".wav"
        self.mp3_file_to_write = "/home/acrv/blaw_ws/src/4_mic_data/mp3_data" + collision_label + start + "_test" + str(test_number) + label + "mic" + str(mic) + mic_position + object_str + ".mp3"
        
        # Check if file exists - if so, terminate and throw error message
        if os.path.exists(self.mp3_file_to_write): 
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
    
    print(topic)
    # Instantiate record_node and spin
    node = record_node(topic, test_number, mic) 
    rospy.spin()  # Start ROS node
