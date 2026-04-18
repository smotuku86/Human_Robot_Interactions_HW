import torch
import torch.nn as nn


# control policy
class Autoencoder(nn.Module):
    def __init__(self, state_dim, hidden_dim, action_dim, latent_dim):
        super(Autoencoder, self).__init__()

        ## define encoder
        # fully connected multi-layer perceptron (MLP)
        # three linear layers
        self.enc_1 = nn.Linear(state_dim+action_dim, hidden_dim)
        self.enc_2 = nn.Linear(hidden_dim, hidden_dim)
        self.enc_3 = nn.Linear(hidden_dim, latent_dim)


        ## define decoder
        # fully connected multi-layer perceptron (MLP)
        # three linear layers
        self.dec_1 = nn.Linear(state_dim+latent_dim, hidden_dim)
        self.dec_2 = nn.Linear(hidden_dim, hidden_dim)
        self.dec_3 = nn.Linear(hidden_dim, action_dim)
        ## helper functions
        # loss function
        self.mse_loss = nn.MSELoss()

    # encoder
    def encoder(self, state, action):
        x = torch.cat((state, action), 1)
        x = torch.tanh(self.enc_1(x))
        x = torch.tanh(self.enc_2(x))

        return 0.1 * torch.tanh(self.enc_3(x)) #bounds to -1:1

    # decoder
    def decoder(self, state, z):
        x = torch.cat((state, z), 1)
        x = torch.tanh(self.dec_1(x))
        x = torch.tanh(self.dec_2(x))

        return 0.1 * torch.tanh(self.dec_3(x)) #this is like velocity - and it shoudl be fine without tanh
    
    # autoencoder
    def forward(self, state, action):
        z = self.encoder(state, action)
        return self.decoder(state, z)
