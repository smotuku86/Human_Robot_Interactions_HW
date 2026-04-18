import torch
import pickle
from models import Autoencoder
from torch.utils.data import Dataset, DataLoader
from pathlib import Path
from tqdm import tqdm

def get_filename_no_ext(filepath):
    return Path(filepath).stem

#train this dataset
dataset_filepath = "HW10/datasets/dataset_50L_movingXY.pkl"
dataset_name = get_filename_no_ext(dataset_filepath)
epoch = 1000
batchsize = 1000
learning_rate = .001
hiddendim = 1024
latentdim = 3

#save trained model here
model_weights_filepath = Path("HW10/model weights") / f"{epoch}_{batchsize}_{learning_rate}_{hiddendim}_{latentdim}_{dataset_name}_weights"

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
def train_model(loadname,  epoch, batchsize, learning_rate, hiddendim, latentdim):
    # select the device to train on
    # use cpu if gpu is not available
    if torch.cuda.is_available():
        DEVICE = torch.device("cuda")
    elif torch.backends.mps.is_available():
        DEVICE = torch.device("mps")
    else:
        DEVICE = torch.device("cpu")

    # training parameters
    print("[-] training autoencoder")
    print(f"[-] device: {DEVICE}")
    EPOCH = epoch
    LR = learning_rate

    # initialize model and optimizer
    model = Autoencoder(state_dim=15, hidden_dim=hiddendim, action_dim=9, latent_dim=latentdim).to(DEVICE)
    optimizer = torch.optim.Adam(model.parameters(), lr=LR)

    # initialize dataset
    print("[-] loading data: " + loadname)
    train_data = MyData(loadname)
    BATCH_SIZE = batchsize #int(len(train_data) / 10.)
    print("my batch size is:", BATCH_SIZE)
    train_set = DataLoader(
        dataset=train_data,
        batch_size=BATCH_SIZE,
        shuffle=True,
        pin_memory=torch.cuda.is_available(),
    )

    # main training loop
    for ep in tqdm(range(EPOCH + 1)):
        for _, x in enumerate(train_set):
            x = x.to(DEVICE, non_blocking=torch.cuda.is_available())
            # collect the demonstrated states and actions
            states = x[:, 0:15]
            actions = x[:, 15:24]
            actions_hat = model(states, actions)

            # compute the loss between actual and predicted
            loss = model.mse_loss(actions, actions_hat)
                 
            # update model parameters
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
        if ep % 1000 == 0:
            print(ep, loss.item())
            torch.save(model.state_dict(), model_weights_filepath)


# train models
if __name__ == "__main__":
    train_model(dataset_filepath, epoch, batchsize, learning_rate, hiddendim, latentdim)