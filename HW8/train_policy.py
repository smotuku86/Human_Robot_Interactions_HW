import torch
import pickle
import numpy as np
from models import MLPPolicy
from torch.utils.data import Dataset, DataLoader
from tqdm import tqdm




import matplotlib.pyplot as plt


# import dataset for training
class MyData(Dataset):

    def __init__(self, loadname):
        self.data = pickle.load(open(loadname, "rb"))
        self.static_images, self.ee_images, self.s_a = map(lambda x: torch.FloatTensor(np.stack(x)), zip(*self.data))
        print("imported dataset of length:", len(self.data))

    def __len__(self):
        return len(self.static_images)

    def __getitem__(self,idx):
        return self.static_images[idx].permute(2, 0, 1), self.ee_images[idx].permute(2, 0, 1), self.s_a[idx]


# train model
def train_model(loadname):
    # select the device to train on
    # use cpu if gpu is not available
    DEVICE = "cpu"
    if torch.cuda.is_available():
        DEVICE = "cuda"
    elif torch.backends.mps.is_available():
        DEVICE = "mps"

    # training parameters
    print(f"[-] training bc using device: {DEVICE}")
    EPOCH = 100
    LR = 0.001

    # initialize model and optimizer
    model = MLPPolicy(state_dim=3, hidden_dim=64, action_dim=3).to(DEVICE)
    optimizer = torch.optim.Adam(model.parameters(), lr=LR)

    # initialize dataset
    print("[-] loading data: " + loadname)
    train_data = MyData(loadname)
    BATCH_SIZE = 64
    print("my batch size is:", BATCH_SIZE)
    train_set = DataLoader(dataset=train_data, batch_size=BATCH_SIZE, shuffle=True)

    # main training loop
    LOSS = []
    for epoch in tqdm(range(EPOCH)):
        for batch_id, batch in enumerate(train_set):
            batch = [k.to(DEVICE) for k in batch]
            static, ee, x = batch

            # collect the demonstrated states and actions
            states = x[:, 0:3]
            actions = x[:, 3:6]
            actions_hat = model(static, ee, states)

            # compute the loss between actual and predicted
            loss = model.mse_loss(actions, actions_hat)
            LOSS.append(loss.item())
                 
            # update model parameters
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
        if epoch % 500 == 0:
            print(epoch, loss.item())
            torch.save(model.state_dict(), "model_weights")
    torch.save(model.state_dict(), "model_weights")

    plt.plot(LOSS)
    plt.show()

# train models
if __name__ == "__main__":
    train_model("dataset.pkl")