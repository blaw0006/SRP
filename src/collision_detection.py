#import sklearn
import matplotlib.pyplot as plt
import librosa
from librosa import load, feature
import torch
from torch import nn
import torchvision
from torchvision import datasets, models, transforms
import numpy as np
from audio_reader import audio_reader, plot
import os

'''
Convolutional Neural Network training and testing code. 

Inputs: 2D representation of audio signals - can use Fast Fourier Transform or Mel spectrogram (CNNs don't use 1D inputs) [6]

Layer structure:
Input -> Conv + Relu -> Pooling -> Conv + Relu -> Pooling -> Fully connected (dense layers) -> Fully connected -> Softmax Output [6]
- Conv layer: applies filter of convolution operation to the input signal to produce feature maps, passes feature maps onto next layer
- Relu activation function converts negative values to 0, enhancing non-linearity of model and preventing over fitting
- Pooling layer used to reduce size of the input while still retaining important information - Max and Avg pooling are common
- Fully connected layer used to connect all neurons in a previous layer to all neurons in the next layer. Used to process the
output of the convolutional layer after the flattening operation
- softmax: output convolutional layer that uses softmax activation function to assign class labels to inputs based upon the 
probability distribution produced by the output layer.

TODO
- check over collision_detection code
- test collision_detection code
- view mel spectrogram images to verify them
- keep in mind that this may not work with weak labelling - may need to adjust or add other layers depending
- finish writing the training properly with the dataloader iterator (nested for loop)
'''
# Construct a model
class collisionDetection(nn.Module):
    '''
    Model for collision detection. The following layers were chosen: 

    '''
    def __init__(self):
        super().__init__()
        # Create nn layers here
        # Will need convolutional layers, pooling layers, a final sigmoid activation function + more
    
    # Define a forward method for forward pass
    def forward(self, x):
        pass


# Define training  and testing loop
def train(model, epochs, train_dataloader, test_dataloader):
    '''
    Inputs
    - model: ResNet18 model
    - epochs: number of epochs to train for
    - train_dataloader: dataloader object containing the training data and labels
    - test_dataloader: dataloader object containing the testing data and labels
    '''

    # define loss function
    loss_fn = nn.BCEloss() # may need BCEWithLogitsLoss(). Examples use cross entropy loss

    # define optimiser
    optimiser = torch.optim.Adam(params=model.parameters, # may need to change optimiser. Examples use sgd
                                 lr=0.01)

    # training loop
    for epoch in epochs:
        # enter model train mode 
        model.train()
        
        for inputs, labels in train_dataloader:
            # Forward pass
            y_logits = model(X_train).squeeze()
            y_pred = torch.round(torch.sigmoid(y_logits))   # Note: May need a different activation function

            # Calculate loss
            loss = loss_fn(y_logits, y_train) # Note: may not be able to use raw logits here depending on loss function

            # Optimiser zero grad
            optimiser.zero_grad()

            # Backpropagation
            loss.backward()

            # Optimiser step
            optimiser.step()

        ### Testing
        model.eval()
        with torch.inference_mode():
            # 1. Forward pass
            test_logits = model(X_test).squeeze()
            test_pred = torch.round(torch.sigmoid(test_logits))

            # 2. Calculate loss/accuracy
            test_loss = loss_fn(test_logits, y_test)

        # Print out what's happening every 10 epochs
        if epoch % 10 == 0:
            print(f"Epoch: {epoch} | Loss: {loss:.5f} | Test loss: {test_loss:.5f}")


def main():
    # Extract time domain audio file 
    # NOTE: the audio_reader processing (converting to decibels over time + bandpass filter) have not been done before this point
    # functionise the filter and use the code here, filtering is important
    
    ####### Pre-processing audio ########
    # Process raw audio and save 
    source = 'src/ur5_control/src/two_mic_tests'
    dest = 'src/ur5_control/src/processed_two_mic_tests'
    spectra_dest = 'src/ur5_control/src/spectra'

    for file in os.listdir(source):
        if file.endswith('.npy'):  # Check it's a numpy file (avoid processing readme files)
            full_source = os.path.join(source, file)
            full_dest = os.path.join(dest, file)
            full_spec_dest = os.path.join(spectra_dest, file)

            # Call audio_reader to open file, process, and save in destination
            time, data = audio_reader(full_source, full_dest, 1) # save processed data 

            # Convert processed data to mel spectrogram and save 
            spectra = feature.melspectrogram(y=data, sr=6300) # NOTE: sampling rate may be incorrect
            np.save(full_spec_dest, spectra)

    #filename = 'C:\\Users\\khash\\OneDrive - Monash University\\Documents\\GitHub\\SRP\\src\\two_mic_tests\\mic1_test1.npy'
    # y, sr = load(filename) # calls librosa.load -> y = np.ndarray representing audio time series, sr = sampling rate
    # y = np.load(filename)
    #spectra = feature.melspectrogram(y=y, sr=22050) # convert np array to mel spectrogram
    #print(spectra)

    # Split data from spectra into a training folder and a test folder
    train_dir = 'src/ur5_control/src/train_data'
    test_dir = 'src/ur5_control/src/test_data'

    # Separate the spectra into these two folders. Note that the folders must have a specific layout for ImageFolder to infer
    # class names. E.g., within train_dir, there must be two folders named 'Class0' and 'Class1' (both are customisable).

    # Resize images with transformation function 
    # NOTE: may need other transforms here as well (normalise?)
    transform = transforms.Compose([
        transforms.Resize((224, 224)),  # resize to 224 * 224
        transforms.CenterCrop(224), # NOTE: may need centre crop or smth else here
        transforms.ToTensor(),  # transform to tensor
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]) # values used for models pretrained on ImageNet
    ])

    # Load and apply transforms
    training_data = datasets.ImageFolder(train_dir, transform) # built in dataset which can be passed as input to a DataLoader
    testing_data = datasets.ImageFolder(test_dir, transform)

    # Dataloader provides an iterable over a torchvision dataset. Each 'batch' in a dataloader is a tuple with {data, label}
    train_dataloader = torch.utils.data.Dataloader(training_data, 
                                                   batch_size=2, # adjust batchsize as needed
                                                   shuffle=True, # shuffling data every epoch reduces overfitting
                                                    )

    testing_dataloader = torch.utils.data.Dataloader(testing_data, 
                                                   batch_size=2, # adjust batchsize as needed
                                                   shuffle=False, 
                                                    )
    
    # Check the current size first !!!!!!

    ####### Machine learning ######
    # Setup device agnostic code 
    device = "cuda" if torch.cuda.is_available() else "cpu"

    # Import pre-trained ResNet-18 
    model = models.resnet18(pretrained=True)

    # Freeze pretrained parameters 
    for param in model.parameters():
        param.requires_grad = False
    
    # Replace fully connected output layer with domain specific layer (two classes so two outputs)
    # NOTE: output layer is now untrained
    in_features = model.fc.in_features
    model.fc = nn.Linear(in_features, 2)
    model.to(device)

    # Run the training loop
    # train(model, epochs=100, train_dataloader, testing_dataloader)


    

if __name__ == "__main__":
    # convert the data to tensor form/spectrograms
    main()
