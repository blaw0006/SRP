#import sklearn
import matplotlib.pyplot as plt
import librosa
from librosa import load, feature
import torch
from torch import nn
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
- check that audio_reader still works
- check over collision_detection code
- test collision_detection code
- research best resnet model to use
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
def training(model, epochs, X_train, y_train, X_test, y_test):
    # need to set manual seed?

    # define loss function
    loss_fn = nn.BCEloss() # may need BCEWithLogitsLoss()

    # define optimiser
    optimiser = torch.optim.Adam(params=model.parameters, # may need to change optimiser
                                 lr=0.01)

    # training loop
    for epoch in epochs:
        model.train()

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
    
    # process audio and save in new folder 
    source = 'src/ur5_control/src/two_mic_tests'
    dest = 'src/ur5_control/src/processed_two_mic_tests'
    spectra_dest = 'src/ur5_control/src/spectra'

    for file in os.listdir(source):
        if file.endswith('.npy'):  # Ensure it's a numpy file    
            full_source = os.path.join(source, file)
            full_dest = os.path.join(dest, file)
            full_spec_dest = os.path.join(spectra_dest, file)

            # Call audio_reader to open file, process, and save in destination
            time, data = audio_reader(full_source, full_dest, 1)

            # Convert processed data to mel spectrogram and save 
            spectra = feature.melspectrogram(y=data, sr=6300) # sampling rate may be incorrect
            np.save(full_spec_dest, spectra)



    #filename = 'C:\\Users\\khash\\OneDrive - Monash University\\Documents\\GitHub\\SRP\\src\\two_mic_tests\\mic1_test1.npy'
    # y, sr = load(filename) # calls librosa.load -> y = np.ndarray representing audio time series, sr = sampling rate
    # y = np.load(filename)
    #spectra = feature.melspectrogram(y=y, sr=22050) # convert np array to mel spectrogram
    #print(spectra)

if __name__ == "__main__":
    # convert the data to tensor form/spectrograms
    main()
