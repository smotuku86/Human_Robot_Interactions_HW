import torch
import torch.nn as nn
from torchvision.models import resnet18, ResNet18_Weights


# vision encoder
class Encoder(nn.Module):
    def __init__(self, image_size=(64, 64), emb_dim=16):
        super(Encoder, self).__init__()

        ## define encoder
        # CNN
        # three layers
        self.encoder = nn.Sequential(nn.Conv2d(3, 24, kernel_size=5, stride=2, padding=0),
                                     nn.ReLU(),
                                     
                                     nn.Conv2d(24, 36, kernel_size=5, stride=2, padding=0),
                                     nn.ReLU(),

                                     nn.Conv2d(36, 48, kernel_size=5, stride=2, padding=0),
                                     nn.ReLU(),

                                     nn.Conv2d(48, 64, kernel_size=3, stride=1, padding=0),
                                     nn.ReLU(),

                                     nn.Conv2d(64, 64, kernel_size=3, stride=1, padding=0),
                                     nn.ReLU(),
                                     nn.Dropout(0.5),
                                     nn.Flatten(start_dim=1))

        # fully connected linear layer for projection
        self.lin_1 = nn.Linear(64, 100)
        self.lin_2 = nn.Linear(100, 50)
        self.lin_3 = nn.Linear(50, emb_dim)

        ## helper functions
        # relu activation function
        self.relu = nn.ReLU()

    ## extract image features
    # input image output encoding
    def forward(self, image):
        image /= 255.
        x = self.encoder(image)
        x = self.relu(self.lin_1(x))
        x = self.relu(self.lin_2(x))
        x = self.lin_3(x)
        return x


# control policy
class MLPPolicy(nn.Module):
    def __init__(self, state_dim, hidden_dim, action_dim, emb_dim=16):
        super(MLPPolicy, self).__init__()
        
        ## initialize vision encoders
        self.static_enc = Encoder()
        self.ee_enc = Encoder()

        ## define policy
        # fully connected multi-layer perceptron (MLP)
        # three linear layers
        self.pi_1 = nn.Linear(state_dim + 2 * emb_dim, hidden_dim)
        self.pi_2 = nn.Linear(hidden_dim, hidden_dim)
        self.pi_3 = nn.Linear(hidden_dim, action_dim)

        ## helper functions
        # relu activation function
        self.relu = nn.ReLU()

    ## execute robot policy
    # input state output action
    def forward(self, static, ee, state):
        z_static = self.static_enc(static)
        z_ee = self.ee_enc(ee)

        x = torch.cat((z_static, z_ee, state), dim=-1)
        x = self.relu(self.pi_1(x))
        x = self.relu(self.pi_2(x))
        return 1.0 * torch.tanh(self.pi_3(x))
