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

<h4> Running capture.launch files </h4>
Under audio_capture/launch/ is a ROS launch file called ```capture.launch```. This file starts multiple instances of the ```audio_capture``` publisher node - one instance for each microphone that you wish to listen to. The launch file does this by specifying the hardware device that each node should publish messages from, in the form of an argument similar to ```plughw:2,0``` where the first number refers to the **card number** of the device, and the second number refers to the **device number**. This particular argument would specify the example audio device listed with ```arecord -l``` in the section above. You must also specify the ```ns``` and ```audio_topic``` arguments. These determine the name of the ROS topic that the audio data will be published to by the audio_capture node - it will be of the form <ns>/<audio_topic>. The other arguments are related to the properties of the audio data to be published and should not be changed unless you are using different microphones.

```roslaunch ur5_control capture.launch``` runs the launchfile. If this doesn't work, try
```roslaunch <catkin_package> capture.launch``` where <catkin_package> is the name of the catkin workspace that the audio_common folder is under. 

Run ```rostopic list``` to check that the topics are being created properly. There should be topics similar to /mic1/audio if you haven't changed the namespace or audio_topic arguments in the launch file. 

```rostopic echo <topic>``` shows the raw data being passed to the specified rostopic. 

Similarly, ```rosparam list``` will list the parameters on the ROS server. 

```rosparam get <parameter>``` will list the parameter being passed to the server. This is a good way of checking that arguments are being passed correctly if you think this may be an issue. In particular, the **device** argument has caused problems in the past, so be careful when changing this.

Note that the launch file is currently setup for four microphones. If you wish to use more, you will need to write another code block specifying the namespace, audio_topic, and device parameters of the new microphones.

<h4> Running record.launch files <h4>
Under src/audio_writer is a ROS launch file called record.launch. This file starts an instance of the subscriber node, record_node.py, for every mic. The file specifies the ROS topic for each node to subscribe to as well as the mic number. The mic number is only relevant for file naming purposes. **test number** is another parameter for file naming. Note that record node will by default search your specified destination filepath (where audio clips will be saved) for the file with the largest test number, and will automatically set the test_number param to the largest test_number + 1. This way the launchfile can be run without specifying the test_number param and it will automatically increment this value for you. **If you wish to specify a test number yourself**, comment out the marked code block in record_node.py and change the default value for test_number in record.launch.

```roslaunch ur5_control record.launch``` or ```roslaunch <catkin_package> capture.launch``` will run the launch file. You can also run 
```roslaunch <catkin_package> capture.launch test_number:="1003"``` if you wish to specify the test number in terminal instead of in the launch file. This requires the marked code block to be commented out in record_node.py. 

If using this file to record, you will likely need to change some filepaths in record_node.py. The ```folder_path``` variable will need to be changed to wherever you will store your mp3 files. Same with the ```self.wav_file_to_write``` and ```self.mp3_file_to_write``` variables. The file assumes that you have folders setup with the following form: 

data
  - mp3_data
      - collision
        - collision cases here
      - no_collision
        - no collision cases here
  - wav_data
    - collision
      - collision cases here
    - no_collision
      - no collision cases here

It may be better to change since you will likely not be working with collision and no_collision cases only.

The file also generates labels for naming files on the assumption that you are dropping objects at one of the 17 marked positions near the UR5 robot, in the numbered order. 

<h3>Training models<h3>
The models and much of the training code was adapted from ______
For my project I took models pretrained on AudioSet and finetuned them on my recorded audio data. To do this, I first downloaded the model from _____.

<h3>Using runme.sh to finetune models<h3>
Under src/panns_transfer_to_gtzan there is a bash script called runme.sh. This file calls the functions defined by the PANNs researchers. The DATASET_DIR is where the mp3 data is located. WORKSPACE is where the training code will save checkpoints. 

The first command in the script calls the ```pack_audio_files_to_hdf5``` function, which converts the mp3 files and stores them in a hierarchical data format. This is how the function stores the data and accesses it in the training code. Alternatively you could write your own training code that converts the mp3 data to mel spectrograms and feeds it to the pretrained model, but this isn't recommended unless you have knowledge of PyTorch and CNNs. 

The second command invoked in ```runme.sh``` calls the training code. Note that the ```Transfer_CNN14_16k``` specified in the model argument was my addition to the ```panns_transfer_to_gtzan/pytorch/models.py``` file. This class initialises the CNN14 class also defined in this file, loads the pretrained model that I have downloaded, and **appends an nn.Linear output layer** to the end. This is how I have adapted the pretrained model for my specific use case. Since I was dealing with collision detection, in which the two output labels are either collision or no_collision, my appended output layer had shape (2048, 2). If you wish to adapt any other PANNs models, you must design your own transfer function. 

Every 200 epochs of training, the code will test the model-in-training against a portion of the dataset and print the validation accuracy to the terminal. 

```./runme.sh``` whilst in the directory that holds the bash script will run it.

<h4>Using inference.sh to validate models<h4>
```CHECKPOINT_PATH``` holds the trained model that you wish to test.
```audio_path``` is the path of the mp3 file that you wish to test the model with. 
Running the code should print the probabilities of each label to the terminal. 


- using ur5_control capture2.launch
- using record.launch and record_node (check the labels and stuff)

Experimental setup

Training models
- need to download PANNS model from here: ____ separately, due to the size of the file
- using runme.sh
- changing the output layer to fit your model
- using inference.sh

Note: trained models are not on the github due to filesize limitations, they are on the UR5 PC