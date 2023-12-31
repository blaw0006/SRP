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

'''
* run audio_common record
* mics publish data to /audio/audio in PCM form 
* subscriber node receives the data -> processes it in some way? Maybe filters data 

Notes
* Sensorised_gripper is a list of functions for the gripper only, there must be other scripts that 
call these functions and control the flow of the robot.  
* Will need to figure out how to move the robot for the simple scenarios

sample_rate: how many times the analog signal sent by a microphone or instrument is sampled per second - 16000


Having multiple input streams
- https://stackoverflow.com/questions/7002423/how-to-mix-multiple-pcm-streams-using-alsa 
- https://github.com/ros-drivers/audio_common/blob/master/sound_play/README.md 
    command for specifying input is at the bottom. Need to start multiple nodes, one for each input to record 
    the data from multiple inputs

To select input device: 
roslaunch audio_capture capture.launch
rosrun sound_play soundplay_node.py _device:="hw:2,0"
rostopic echo /audio/audio

To run this node:
rosrun ur5_control audio_handler.py

Note: something broke after selecting input device as above, its possible the default device was changed 
because data is no longer being received. Will need to fix this on monday. 
Update: no longer have this issue despite not changing anything. Might be a hardware issue, or the default 
device was changed back after restarting the computer 

Running multiple nodes for multiple devices
roslaunch audio_capture capture.launch device:="" ns:="bruh" seems to work when device is not specified (uses default)
However, specifying device:="hw:2,0" raises a gstreamer: Internal data stream error
- Update: need an underscore in front of the device, e.g., roslaunch audio_capture capture.launch _device:="hw:3,0" ns:="t1"
- However, this also seems to record data when a nonexistent device (hw:10,0) is used

Converting AudioData Ros message to usable form
- np.from_buffer can be used to extract the numerical data as a numpy array of 8 bit unsigned integers,
without all of the buffers
    - however, existing mp3 to wav converters require the inputs to be in a specific form 
- need to try using pydub with the raw data.data format

Runtime issues
- Time taken to process and plot data > time between new ROS messages
- leads to multiple instances of the callback function calling the visualise method at a time, leading to 
threading errors since matplotlib does not allow multiple threads (that are not main thread) to be calling
plt functions
- using threading.lock doesnt seem to work, terminal freezes after printing two datasets, suggesting that one
of the threads is holding onto the lock and preventing other threads from using it
    - regardless, expect issues since even if the lock worked, the threads would queue up to edit the plot, 
    which would lead to delays between sounds heard and seen on soundwave
'''
class audio_visualiser:
    def __init__(self):
        rospy.init_node('audio_handler', anonymous=True)
        rospy.Subscriber('/t1/audio', inputMsg, self.audio_callback)
        #self.audio_data = None
        self.pcm = None
        self.fig, self.axis = plt.subplots() # initialises figure and subplot
        plt.ion()   # turns on interactive mode for live data display (figures will be shown immediately upon creation)
        
        self.lock = Lock() # create threadlock for thread synchronisation
        
        # call visualisation method from main thread at a set frequency
        self.vis_timer = rospy.Timer(rospy.Duration(0.5), self.trigger_vis)
        
        
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
        print(self.pcm)
        
        # Call visualise method for plotting
        #self.visualise_audio()
        
        # plot data to check if the raw data is valid,
        # also try writing to a file to save it (for data collection purposes)
        # next step is to allow running of this node for multiple inputs
        # what is the difference between calling the visualise function in callback, and in if name == main? 
        # Why is only one set of values being plotted despite the pcm data changing constantly? Either
        # the function is only being called once in this scenario, or variable is not being updated (but it should be since instance variable)
    
        #rospy.sleep(1) # check what the sleep function does
        
    def trigger_vis(self, event):
        # calls the visualise audio method in the main thread
        rospy.run_in_thread(self.visualise_audio)
    
    def visualise_audio(self): 
        # plots the current converted pcm data held in self.pcm onto the same plot
        #while not rospy.is_shutdown(): # checks that shutdown flag is False
        try:
            if self.pcm is not None: # checks that data has been processed before running
                with self.lock: # check that lock has been acquired (no other thread is plotting)
                    time = np.arange(0,len(self.pcm))/16000 # size of pcm np_array is variable 
                    
                    # Plotting and titles
                    self.axis.plot(time, self.pcm)
                    self.axis.set_title('Audio Waveform')
                    self.axis.set_xlabel('Time (seconds)')
                    self.axis.set_ylabel('Amplitude')
                    self.fig.canvas.draw() # redraws the current figure. May not be needed since plt.ion is active
                    self.fig.canvas.flush_events() # handles events (not needed tbh)
                    #plt.pause(0.001)
                    
        except Exception as e:
            print("Exception in visualise_audio:", e)
                    

        
        
    
if __name__ == '__main__':
    vis = audio_visualiser()    

    # vis.visualise_audio() # calls the visualise method explicitly + separately from the callback. Calling 
    # within the callback makes more sense here since the visualisation is tied to the data being processed in callback
    
    # keeps the node running until interrupted (ctrl-c)
    rospy.spin()
    
    
