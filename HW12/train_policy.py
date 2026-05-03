import torch
import pickle
from models import MLPPolicy
from torch.utils.data import Dataset, DataLoader


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
def train_model(loadname, savenumber):

    # training parameters
    print("[-] training bc")
    EPOCH = 500
    LR = 0.001

    # initialize model and optimizer
    model = MLPPolicy(state_dim=4, hidden_dim=64, action_dim=2)
    optimizer = torch.optim.Adam(model.parameters(), lr=LR)

    # initialize dataset
    print("[-] loading data: " + loadname)
    train_data = MyData(loadname)
    BATCH_SIZE = max(10, round(len(train_data) / 10.))
    print("my batch size is:", BATCH_SIZE)
    train_set = DataLoader(dataset=train_data, batch_size=BATCH_SIZE, shuffle=True)

    # main training loop
    for epoch in range(EPOCH+1):
        for batch, x in enumerate(train_set):
        
            # collect the demonstrated states and actions
            states = x[:, 0:4]
            actions = x[:, 4:6]
            actions_hat = model(states)

            # compute the loss between actual and predicted
            loss = model.mse_loss(actions, actions_hat)
                 
            # update model parameters
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
        if epoch % 10 == 0:
            print(epoch, loss.item())
            torch.save(model.state_dict(), "model_weights" + str(savenumber))

# train models
if __name__ == "__main__":
    train_model("dataset.pkl", 0)