import pybullet as p
import pybullet_data
import numpy as np
import os
import time
import torch
from models import MLPPolicy
from robot import Panda
from tqdm import tqdm

import matplotlib.pyplot as plt


# parameters
control_dt = 1. / 240.

# create simulation and place camera
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.resetDebugVisualizerCamera(cameraDistance=1.0, 
                                cameraYaw=40.0,
                                cameraPitch=-30.0, 
                                cameraTargetPosition=[0.5, 0.0, 0.2])

# load the objects
urdfRootPath = pybullet_data.getDataPath()
plane = p.loadURDF(os.path.join(urdfRootPath, "plane.urdf"), basePosition=[0, 0, -0.625])
table = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[0.5, 0, -0.625])
cube = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), basePosition=[0.5, 0, 0.025])
p.changeVisualShape(cube, -1, rgbaColor=[1, 0, 0, 1])      # change cube color

# load the robot
jointStartPositions = [0.0, 0.0, 0.0, -2*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.0, 0.0, 0.04, 0.04]
panda = Panda(basePosition=[0, 0, 0],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                jointStartPositions=jointStartPositions)

# select the device to train on
# use cpu if gpu is not available
DEVICE = "cpu"
if torch.cuda.is_available():
    DEVICE = "cuda"
elif torch.backends.mps.is_available():
    DEVICE = "mps"
    
# load the trained model
model = MLPPolicy(state_dim=3, hidden_dim=64, action_dim=3).to(DEVICE)
model.load_state_dict(torch.load('model_weights'))
model.eval()

# test and see how your learned policy does!
n_tests = 10
action_magnitude = 1.0
for test_idx in tqdm(range(n_tests)):

    # reset the robot
    panda.reset(jointStartPositions)
    cube_position = np.random.uniform([0.3, -0.3, 0.025], [0.7, +0.3, 0.025])
    p.resetBasePositionAndOrientation(cube, cube_position, p.getQuaternionFromEuler([0, 0, 0]))

    # run sequence of position and gripper commands
    traj = []
    for time_idx in range (1000):

        # get the robot's position
        robot_state = panda.get_state()
        robot_pos = np.array(robot_state["ee-position"])
        traj.append(robot_pos)

        # get the state
        state = torch.FloatTensor(robot_pos).to(DEVICE)
        static = torch.FloatTensor(robot_state["static"]).to(DEVICE).permute(2, 0, 1)
        ee = torch.FloatTensor(robot_state["ee"]).to(DEVICE).permute(2, 0, 1)

        # use the learned policy to output an action
        action = model(static[None, :], ee[None, :], state[None, :]).detach().cpu().numpy().squeeze()

        # normalize the size of the action
        if np.linalg.norm(action) > action_magnitude:
            action *= action_magnitude / np.linalg.norm(action)

        # move the robot with action
        panda.move_to_pose(robot_pos + action, ee_rotz=0, positionGain=0.01)
        p.stepSimulation()
        time.sleep(control_dt)


        cube_position = np.random.uniform([0.3, -0.3, 0.025], [0.7, +0.3, 0.025])        
        if np.linalg.norm(robot_pos - cube_position) < 0.01:
            print('success')
            break
    
    # traj = np.stack(traj)
    # _, ax = plt.subplots(1, 1, subplot_kw={'projection': '3d'})
    # ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 'k-')
    # ax.scatter(cube_position[0], cube_position[1], cube_position[2], c='b', s=20)
    # plt.show()
