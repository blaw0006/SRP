<h3>Requirements and setup</h3>
Data collection was integrated with ROS as much as possible and requires:
- ROS Melodic 
- Ubuntu 18.04
- Python 2.7

Training code requires a later version of Python and ROS:
- ROS Noetic
- Ubuntu 20.04
- Python 3+

Follow online ROS tutorials to setup ROS and the Catkin workspace correctly if needed
- http://wiki.ros.org/noetic/Installation/Ubuntu
- http://wiki.ros.org/catkin/Tutorials

If working on a fresh Ubuntu installation, you will have to install many packages to run this code. The system will tell you what is needed when 
you attempt to run the commands in terminal. Installation methods can sometimes differ, but generally use
```pip install <package_name>``` or ```pip install <package_name>==<version>``` if you require a specific version. In some cases you will need an 
older version of a package, especially when working with ROS Melodic - here you should use the second command to install whatever version the system says is required. 

<h3>Recording</h3>
<h4> Connecting microphones </h4>
This section will outline how to record audio clips with the contact microphones. Connect microphones -> usb-to-jack adaptor -> usb dock -> PC. 
Run ```arecord -l``` to check that the devices are connected properly. This commands lists all available **input** devices. The devices will appear similar to:

card 2: Device [USB PnP Sound Device], device 0: USB Audio [USB Audio]
  Subdevices: 1/1
  Subdevice #0: subdevice #0

The first two devices typically correspond to the PC's in-built microphones, whilst usb connected devices will have "USB Pnp Sound Device" or "USB Audio device in their name".

<h4> Running launch files </h4>
Under audio_capture/launch/ is a ROS launch file called ```capture.launch```. This file starts multiple instances of the ```audio_capture``` publisher node - one instance for each microphone that you wish to listen to. The launch file does this by specifying the hardware device that each node should publish messages from, in the form of an argument similar to ```plughw:2,0``` where the first number refers to the **card number** of the device, and the second number refers to the **device number**. This code would specify the example audio device listed with ```arecord -l``` in the section above. You must also specify the ```ns``` and ```audio_topic``` arguments. These determine the name of the ROS topic that the audio data will be published to by the audio_capture node - it will be of the form <ns>/<audio_topic>. The other arguments are related to the properties of the audio data to be published and should not be changed unless you are using different microphones.

```roslaunch ur5_control capture.launch``` runs the launchfile. If this doesn't work, try
```roslaunch <catkin_package> capture.launch``` where <catkin_package> is the name of the catkin workspace that the audio_common folder is under. 

Run ```rostopic list``` to check that the topics are being created properly. There should be topics similar to /mic1/audio if you haven't changed the namespace or audio_topic arguments in the launch file. 

```rostopic echo <topic>``` shows the raw data being passed to the specified rostopic. 

Similarly, ```rosparam list``` will list the parameters on the ROS server. 

```rosparam get <parameter>``` will list the parameter being passed to the server. This is a good way of checking that arguments are being passed correctly if you think this may be an issue. In particular, the **device** argument has caused problems in the past, so be careful when changing this.



```
- using ur5_control capture2.launch
- using record.launch and record_node (check the labels and stuff)

Experimental setup

Training models
- need to download PANNS model from here: ____ separately, due to the size of the file
- using runme.sh
- changing the output layer to fit your model
- using inference.sh

Note: trained models are not on the github due to filesize limitations, they are on the UR5 PC