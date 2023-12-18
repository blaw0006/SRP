#! /usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from pydub import AudioSegment
from audio_common_msgs.msg import AudioData as inputMsg
from io import BytesIO
from StringIO import StringIO
import threading

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
'''
class audio_visualiser:
    def __init__(self):
        rospy.init_node('audio_handler', anonymous=True)
        rospy.Subscriber('/t1/audio', inputMsg, self.audio_callback)
        self.audio_data = None
        self.pcm = None
        self.fig, self.axis = plt.subplots() # initialises figure and subplot
        plt.ion()   # turns on interactive mode for live data display
        self.lock = threading.Lock()
        
        
    def audio_callback(self, data):
        # save data
        # process data 
        # save maximum?
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data.data))
        # append data and print to terminal?
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data.data[:10]))
        #print(data)
        #print(data.data)
        #print(len(data.data))
        #print(type(data.data))
        #print(int(data.data))
        #raw_input()
        
        #self.audio_data = data
        #self.np_array = np.array(self.audio_data)
        
        #print(self.np_array)
        #print(len(self.np_array))
        #print(self.np_array[0])
        #print(self.np_array.shape)
        
        
        #print(self.audio_data)
        #print(len(self.audio_data))
        #print(type(self.audio_data)) # data is of type class 'audio_common_msgs.msg._AudioData.AudioData'
        #print(np.array(self.audio_data)) # has the same effect as above 
        #print(type(np.array(self.audio_data)))
        
        #------------------------------
        audio_samples = data.data # need the .data field to access the mp3 data (hexadecimal bytestring of the form '\xff\xf3\xb8\xc4\r}d\x15\xd8')
        
        #audio_samples = np.array(audio_samples, dtype=np.uint8)
        #audio_samples = audio_samples.tobytes()
        # converts bytes to numpy array of 8 bit unsigned integers
        #self.audio_data = np.frombuffer(audio_samples, dtype=np.uint8) # extracts numerical data from buffered data
        
        #self.audio_data = bytes(audio_samples)
        #audio_segment = AudioSegment.from_file(BytesIO(self.audio_data.tobytes()), format="mp3") 
        #audio_segment = AudioSegment.from_mp3(StringIO(audio_samples)) 
        
        '''
        AudioSegment accepts either file path to mp3 file or raw mp3 data (in the form of bytestrings) as input
        Since data.data accesses the raw audio data as a bytestring, no further processing is needed to convert
        it to a bytestring (np.array(input) and np_array.tobytes() do not have to be used).
        '''
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
        #self.visualise_audio()
        
        # plot data to check if the raw data is valid,
        # also try writing to a file to save it (for data collection purposes)
        # next step is to allow running of this node for multiple inputs
        # what is the difference between calling the visualise function in callback, and in if name == main? 
        # Why is only one set of values being plotted despite the pcm data changing constantly? Either
        # the function is only being called once in this scenario, or variable is not being updated (but it should be since instance variable)
        
        
        #mp3_bytes = bytes(self.audio_data)
        
        #print(audio_segment)
        #print(self.audio_data)
        #print(self.audio_data.shape)
        
        #print(self.audio_data.shape)
        #mp3_data = np.frombuffer(data, dtype=np.uint8)
        #print(mp3_data)
        
        
        
        self.visualise_audio()
        #rospy.sleep(1) # check what the sleep function does
        
        
    
    def visualise_audio(self): 
        # plots the current converted pcm data held in self.pcm onto the same plot
        #while not rospy.is_shutdown(): # checks that shutdown flag is False
        with self.lock:
            if self.pcm is not None: # checks that data has been processed before running
                #audio_array = np.array(self.pcm)
                #time = np.arange(0, len(audio_array)) / 16000
                time = np.arange(0,len(self.pcm))/16000 # size of pcm np_array is variable 

                self.axis.plot(time, self.pcm)
                self.axis.set_title('Audio Waveform')
                self.axis.set_xlabel('Time (seconds)')
                self.axis.set_ylabel('Amplitude')
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
                #plt.pause(0.001)
            
            '''
            plt.figure()
            plt.plot(time, self.pcm)
            plt.title('Audio Waveform')
            plt.xlabel('Time (seconds)')
            plt.ylabel('Amplitude')
            plt.show()
            '''
        
        
    
if __name__ == '__main__':
    vis = audio_visualiser()    

    # vis.visualise_audio() # calls the visualise method explicitly + separately from the callback. Calling 
    # within the callback makes more sense here since the visualisation is tied to the data being processed in callback
    
    '''
    bruh = []
    bruh_array = np.array(bruh)
    print(bruh_array)
    print(bruh_array.shape)
    rospy.spin()
    '''
    
    # keeps the node running until interrupted (ctrl-c)
    rospy.spin()
    
    
