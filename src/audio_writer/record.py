#! /usr/bin/env python
import rospy
import numpy as np
#import matplotlib.pyplot as plt
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from pydub import AudioSegment
from audio_common_msgs.msg import AudioData as inputMsg
from io import BytesIO
from StringIO import StringIO
from threading import Lock
import time
import os
import sys
from record_mp3 import record_mp3
from record_wavmp3 import record_wavmp3
from record_np import record_np

if __name__ == '__main__': 
    # Initialise important values 
    num_mics = 4 # change based on number of mics connected to system
    label = "/collision" # collision clips - affects the path location
    start = "/1" # heading for the clips
    
    # important, used to create or overwrite file for recorded sound
    test = str(input("Enter test number: "))
    
    # creates a sound file for each mic
    '''
    # record_np 
    file1 = "src/ur5_control/src/two_mic_tests/mic1_test" + test + ".npy"
    file2 = "src/ur5_control/src/two_mic_tests/mic2_test" + test + ".npy"
    '''
    
    wav_files = [None] * num_mics
    mp3_files = [None] * num_mics
    
    for i in range(1, num_mics+1):
        wav_files[i-1] = "/home/acrv/blaw_ws/src/wav_data" + label + start + "mic" + str(i) +"_test" + test + ".wav"
        mp3_files[i-1] = "/home/acrv/blaw_ws/src/mp3_data" + label + start + "mic" + str(i) + "_test" + test + ".mp3"
    
    # Check if file exists - if so, terminate and throw error message
    if os.path.exists(wav_files[0]): 
        override = int(input("Test already exists. Enter 1 to overwrite: "))
        if override != 1:
            sys.exit()
    
    # start timing
    start = time.time()
    
    # starts subscriber node for each topic - MUST name the namespaces as mic1 and mic2 when roslaunching audio_common
    wav_subscribers = [None] * num_mics
    mp3_subscribers = [None] * num_mics
    
    for i in range(1, num_mics + 1):
        topic = "/mic" + str(i) + "/audio"
        print(wav_files[i-1])
        print(mp3_files[i-1])
        wav_subscribers[i-1] = record_wavmp3(topic, wav_files[i-1], mp3_files[i-1])
        #mp3_subscribers[i-1] = record_mp3(topic, mp3_files[i-1])
    
    for i in range(1, num_mics + 1):
        rospy.on_shutdown(wav_subscribers[i-1].shutdown_callback)
        #rospy.on_shutdown(mp3_subscribers[i-1].shutdown_callback)
        
    # keeps the node running until interrupted (ctrl-c)
    rospy.spin()
    
    # print time
    end = time.time()
    print("Time elapsed: ")
    print(end-start)
    # 6326 6298 6491 6500 6200 6400