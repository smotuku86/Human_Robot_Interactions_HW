import torch
import numpy as np
from models import MLPPolicy
from main import dynamics
import matplotlib.pyplot as plt


# load the trained model
model0 = MLPPolicy(state_dim=4, hidden_dim=64, action_dim=2)
model0.load_state_dict(torch.load('model_weights0'))
model0.eval()

# plot the trajectory the mobile robot executes
def plot_trajectory(xi):
    plt.plot(xi[:,0], xi[:,1], 'bo-')
    plt.plot(xi[0,0], xi[0,1], 'ks')
    plt.plot(xi[0,2], xi[0,3], 'ro')
    plt.axis("equal")
    plt.savefig("xi.png")

# main loop
n_tests = 100
average_goal_error = 0
for idx in range(n_tests):

    # reset the system
    initial_state = np.random.uniform(-10, 10, 4)
    state = np.copy(initial_state)

    # rollout the learned policy
    xi = [state]
    for idx in range(20):
        
        # get action from model
        state_tensor = torch.FloatTensor(state)
        action = model0(state_tensor).detach().numpy()

        # compute next state
        state = dynamics(state, action)
        xi.append(state)
        
    # record the error
    average_goal_error += np.linalg.norm(state[0:2]-state[2:4]) / n_tests

    # # plot the trajectory
    # xi = np.array(xi)
    # plot_trajectory(xi)

print("here is my average error from the closest goal:", np.round(average_goal_error, 2))