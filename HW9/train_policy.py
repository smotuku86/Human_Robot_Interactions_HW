import torch
import pickle
from models import MLPPolicy
from torch.utils.data import Dataset, DataLoader
from pathlib import Path

def get_filename_no_ext(filepath):
    return Path(filepath).stem

#train this dataset
dataset_filepath = "HW9/datasets/dataset_history_upsampled.pkl"
dataset_name = get_filename_no_ext(dataset_filepath)
epoch = 700
batchsize = 50
learning_rate = .01
hiddendim = 64

#save trained model here
model_weights_filepath = Path("HW9/model_weights") / f"{epoch}_{batchsize}_{learning_rate}_{hiddendim}_upsampled_datasetwhistory_weights"

# import dataset for training
class MyData(Dataset):

    def __init__(self, loadname):
        self.data = pickle.load(open(loadname, "rb"))
        self.data = torch.FloatTensor(self.data)
        print("imported dataset of length:", len(self.data))

    def __len__(self):
        return len(self.data)

    def __getitem__(self,idx):
        return self.data[idx]


# train model
def train_model(loadname, epoch, batchsize, learning_rate, hiddendim):

    # training parameters
    print("[-] training bc")
    EPOCH = epoch
    LR = learning_rate

    # initialize model and optimizer
    model = MLPPolicy(state_dim=12, hidden_dim=hiddendim, action_dim=3)
    optimizer = torch.optim.Adam(model.parameters(), lr=LR)

    # initialize dataset
    print("[-] loading data: " + loadname)
    train_data = MyData(loadname)
    BATCH_SIZE = batchsize
    print("my batch size is:", BATCH_SIZE)
    train_set = DataLoader(dataset=train_data, batch_size=BATCH_SIZE, shuffle=True)

    # main training loop
    for epoch in range(EPOCH+1):
        for batch, x in enumerate(train_set):
        
            # collect the demonstrated states and actions
            # states = x[:, 0:6]
            # actions = x[:, 6:9]
            states = x[:, 0:12]
            actions = x[:, 12:15]
            actions_hat = model(states)

            # compute the loss between actual and predicted
            loss = model.mse_loss(actions, actions_hat)
                 
            # update model parameters
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
        if epoch % 10 == 0:
            print(epoch, loss.item())
            torch.save(model.state_dict(), model_weights_filepath)

# train models
if __name__ == "__main__":
    train_model(dataset_filepath, epoch, batchsize, learning_rate, hiddendim)