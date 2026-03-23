import torch
import torch.nn as nn


# control policy
class MLPPolicy(nn.Module):
    def __init__(self, state_dim, hidden_dim, action_dim):
        super(MLPPolicy, self).__init__()

        ## define policy
        # fully connected multi-layer perceptron (MLP)
        # three linear layers
        self.pi_1 = nn.Linear(state_dim, hidden_dim)
        self.pi_2 = nn.Linear(hidden_dim, hidden_dim)
        self.pi_3 = nn.Linear(hidden_dim, action_dim)

        ## helper functions
        # relu activation function
        self.relu = nn.ReLU()
        # loss function
        self.mse_loss = nn.MSELoss()

    ## execute robot policy
    # input state output action
    def forward(self, state):
        x = self.relu(self.pi_1(state))
        x = self.relu(self.pi_2(x))
        return 0.1 * torch.tanh(self.pi_3(x))