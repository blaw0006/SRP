import sklearn
import matplotlib as plt
import torch
from torch import nn

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
        pass

if __name__ == "__main__":
    # convert the data to tensor form/spectrograms
    pass