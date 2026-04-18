import pybullet as p
import pybullet_data
import numpy as np
import os
import time
from robot import Panda
from objects import objects
from models import Autoencoder
from teleop import KeyboardController
import torch


#model weights
model_weights_filepath = "HW10/model weights/1000_100_0.001_128_6_dataset_50L_movingXY_weights"

# parameters
control_dt = 1. / 240.

# create simulation and place camera
physicsClient = p.connect(p.GUI)
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
cylinder_position = np.random.uniform([0.6, -0.2, 0.1], [0.6, +0.2, 0.1])
cylinder = objects.SimpleObject("cylinder.urdf", basePosition=cylinder_position)

# load the robot
jointStartPositions = [0.0, 0.0, 0.0, -2*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.0, 0.0, 0.04, 0.04]
panda = Panda(basePosition=[0, 0, 0],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                jointStartPositions=jointStartPositions)

# load the trained model
model = Autoencoder(state_dim=15, hidden_dim=128, action_dim=9, latent_dim=6)
model.load_state_dict(torch.load(model_weights_filepath))
model.eval()

# teleoperation interface
teleop = KeyboardController()

# main loop
gripper_state = [-1.0]
while True:
    # get the robot's position
    robot_state = panda.get_state()
    robot_pos = np.array(robot_state["joint-position"])

    # get user input
    action = teleop.get_action()

    # open or close the gripper
    if action[6] == +1:
        panda.open_gripper()
        gripper_state = [-1.0]
    elif action[6] == -1:
        panda.close_gripper()
        gripper_state = [+1.0]

    # exit current simulation
    if action[7]:
        break

    # get the full state
    state = robot_pos.tolist() + gripper_state + cylinder_position.tolist()
    disp2cyl = np.linalg.norm(np.array(robot_state['ee-position']) - cylinder_position)
    #print("disp2cyl:", disp2cyl, type(disp2cyl), np.shape(disp2cyl))
    # use the learned policy to output an action
    state_t = torch.FloatTensor([state])
    z_t = torch.FloatTensor([[500 * action[0], gripper_state[0], disp2cyl]])
    action = 0.1 * model.decoder(state_t, z_t).detach().numpy()[0]


    # move the robot with action
    panda.move_to_joint(robot_pos[0:9] + action)
    p.stepSimulation()
    time.sleep(control_dt)