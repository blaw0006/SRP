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

'''
Writer node that extracts data from rostopic and saves it
- converted data is in np array format, which cannot be written directly into a txt file
- np.savez cannot be used to add multiple arrays to the same npz file on separate calls, must all be in one call
- instead, attempted to append + save all data into an instance variable, then add it to an np file in one go

TODO
- allow running of this node for multiple inputs (setup roslaunch file that runs several nodes that subscribe to different topics)
- choose correct rate to divide np.arange by 
'''
class audio_visualiser:
    def __init__(self):
        rospy.init_node('audio_handler', anonymous=True)
        rospy.Subscriber('/t1/audio', inputMsg, self.audio_callback)
        #self.audio_data = None
        self.pcm = None 
        self.data = np.array([]) # initialise empty np array
        #self.count = np.array(0) # counter to be used as key for np.savez input
        self.count = 0
        
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
        
        #self.pcm = np.array(audio_segment.get_array_of_samples())
        audio_segment.export("audio_output.wav", format="wav") # export to wav format 
        
        # hidden instance variable containing raw wav data after conversion 
        wav_data = audio_segment._data
        
        # convert wav to np_array for plotting
        self.pcm = np.frombuffer(wav_data, dtype=np.int16) 
        #print(self.pcm)
        
        # write numpy data to file (as binary np, can be loaded to np arrays later)
        # np.savez("filepath", np_array, key)
        #np.savez("src/ur5_control/src/bruh.npz", self.pcm, self.count)
        #np.savez("src/ur5_control/src/bruh.npz", **{str(int(self.count)): self.pcm})
        #key = f'array_{self.count}'
        
        #key = 'array_{}'.format(self.count)
        #np.savez("src/ur5_control/src/bruh.npz", **{key: self.pcm})
        self.data = np.append(self.data, self.pcm)
        self.count += 1
        
        
        # Call visualise method for plotting
        #self.visualise_audio()
        
       
    
    def shutdown_callback(self):
        print(self.data)
        print(np.shape(self.data))
        #np.savez("src/ur5_control/src/bruh.npz", self.data, self.count)
        np.save("src/ur5_control/src/bruh.npy", self.data)
        
        # stop timing
        end = time.time()
        print(end - start)
        
    
if __name__ == '__main__':
    # start timing
    start = time.time()
    
    vis = audio_visualiser()    

    # vis.visualise_audio() # calls the visualise method explicitly + separately from the callback. Calling 
    # within the callback makes more sense here since the visualisation is tied to the data being processed in callback
    
    
    # save appended data arrays after shutdown
    rospy.on_shutdown(vis.shutdown_callback)
    
    
    
    
    # keeps the node running until interrupted (ctrl-c)
    rospy.spin()
    
   
    # 6326 6298 6491 6500 6200 6400
    
    
