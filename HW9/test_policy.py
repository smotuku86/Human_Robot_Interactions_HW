import pybullet as p
import pybullet_data
import numpy as np
import os
import time
from robot import Panda
from objects import objects
from models import MLPPolicy
import torch
from tqdm import tqdm

model_weights_filepath = "HW9/model_weights/1000_100_0.001_32_upsampled_dataset_weights"

# parameters
control_dt = 1. / 240.

# create simulation and place camera
physicsClient = p.connect(p.DIRECT)
p.setGravity(0, 0, -9.81)
# disable keyboard shortcuts so they do not interfere with keyboard control
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.resetDebugVisualizerCamera(cameraDistance=1.0, 
                                cameraYaw=40.0,
                                cameraPitch=-40.0, 
                                cameraTargetPosition=[0.5, 0.0, 0.2])

# load the objects
urdfRootPath = pybullet_data.getDataPath()
plane = p.loadURDF(os.path.join(urdfRootPath, "plane.urdf"), basePosition=[0, 0, -0.625])
table = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[0.5, 0, -0.625])
cabinet = objects.CollabObject("cabinet.urdf", basePosition=[0.8, 0, 0.2], baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi]))
p.resetJointState(cabinet.object, 0, 0.1)

# load the robot
jointStartPositions = [0.0, 0.0, 0.0, -2*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.0, 0.0, 0.04, 0.04]
panda = Panda(basePosition=[0, 0, 0],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                jointStartPositions=jointStartPositions)

# load the trained model
model = MLPPolicy(state_dim=6, hidden_dim=32, action_dim=3)
model.load_state_dict(torch.load(model_weights_filepath))
model.eval()
score = 0
n_tests = 10
# main loop
for idx in tqdm(range(n_tests)):

    # reset the robot
    panda.reset(jointStartPositions)
    cabinet_position = np.random.uniform([0.6, -0.3, 0.2], [0.8, +0.3, 0.2])
    p.resetBasePositionAndOrientation(cabinet.object, cabinet_position, p.getQuaternionFromEuler([0, 0, np.pi]))
    p.resetJointState(cabinet.object, 0, 0.1)
    robot_state = panda.get_state()
    prev_pos = np.array(robot_state["ee-position"])
    robot_positions = []
    robot_positions.append(np.copy(prev_pos))
    robot_positions.append(np.copy(prev_pos))

    #get initial pos of cabinet handle 
    initial_cabinet_state = cabinet.get_state()
    initial_cabinet_handle_position = initial_cabinet_state["handle_position"][0] # get x cord - it only travels in that direction
    handle_displacement = 0
    # rollout the learned policy
    for idx in range(2000):
        # get the robot's position
        robot_state = panda.get_state()
        robot_pos = np.array(robot_state["ee-position"])
        if idx % 10 == 0:
            robot_positions.append(np.copy(robot_pos))
        robot_pos_history = robot_pos.tolist() + robot_positions[-1].tolist() +  robot_positions[-2].tolist()
        #use above line for passing prev states to model
        # get the state
        state = torch.FloatTensor(robot_pos.tolist() + cabinet_position.tolist())

        # use the learned policy to output an action
        action = 10 * model(torch.FloatTensor(state)).detach().numpy()

        # move the robot with action
        panda.move_to_pose(robot_pos + action, ee_rotz=0, positionGain=0.01)

        #check if it completed task of opening cabinet
        current_cabinet_state = cabinet.get_state()
        cabinet_handle_position = current_cabinet_state["handle_position"][0]
        handle_displacement = abs(cabinet_handle_position - initial_cabinet_handle_position)
                                # shouldn't use abs bc if it goes back it could count
        #print(handle_displacement)
        p.stepSimulation()
        time.sleep(control_dt)

    #at the end, increment score
    score += min(1, handle_displacement / .05) #hardcode alert
    # this is the most the handle can go out from .1 extentsion

print("Score:", round(score/n_tests * 100,2))