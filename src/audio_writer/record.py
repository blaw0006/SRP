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
from record_mp3.py import record_mp3
from record_wav.py import record_wav
from record_np.py import record_np

if __name__ == '__main__':
    print("Select function: 2-audio_visualiser, 1-audio_visualiserV2, 0-record_mp3")
    visualise = int(input("Enter function number: ")) # visualise = 0 --> run record_mp3
    
    # start timing
    start = time.time()
    
    # important, used to create or overwrite file for recorded sound
    test = str(input("Enter test number: "))
    
    # creates a sound file for each mic
    # visualiser 1 files
    file1 = "src/ur5_control/src/two_mic_tests/mic1_test" + test + ".npy"
    file2 = "src/ur5_control/src/two_mic_tests/mic2_test" + test + ".npy"

    # visualiser 2 files (wav and np)
    file3 = "src/ur5_control/src/two_mic_tests/mic1_test" + test + ".wav"
    file4 = "src/ur5_control/src/two_mic_tests/mic2_test" + test + ".wav"
    
    # record_mp3 files - currently set to no collision !!!!
    file5 = "/home/acrv/blaw_ws/src/ur5_control/src/SRP/src/mp3_test/no_collision/0mic1_test" + test + ".mp3"
    file6 = "/home/acrv/blaw_ws/src/ur5_control/src/SRP/src/mp3_test/no_collision/0mic2_test" + test + ".mp3"
    
    
    # starts subscriber node for each topic - MUST name the namespaces as t1 and t2 when roslaunching audio_common
    
    # choose if audio_visualiser or record_mp3 is used
    if visualise == 2: # original visualiser function
        vis1 = audio_visualiser('/t1/audio', file1)    
        vis2 = audio_visualiser('/t2/audio', file2)
    elif visualise == 1: # visualiser v2 function
        vis1 = audio_visualiserV2('/t1/audio', file1, file3)
        vis2 = audio_visualiserV2('/t2/audio', file2, file4)
    else:  # record mp3 function
        vis1 = record_mp3('/t1/audio', file5)
        vis2 = record_mp3('/t2/audio', file6)

    # vis.visualise_audio() # calls the visualise method explicitly + separately from the callback. Calling 
    # within the callback makes more sense here since the visualisation is tied to the data being processed in callback
    
    
    # save appended data arrays after shutdown
    rospy.on_shutdown(vis1.shutdown_callback)
    rospy.on_shutdown(vis2.shutdown_callback)
    
    
    # keeps the node running until interrupted (ctrl-c)
    rospy.spin()
    
   
    # 6326 6298 6491 6500 6200 6400