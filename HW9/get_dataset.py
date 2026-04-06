import pybullet as p
import pybullet_data
import numpy as np
import os
import time
from robot import Panda
from objects import objects
import pickle

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
cabinet = objects.CollabObject("cabinet.urdf", basePosition=[0.8, 0, 0.2], baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi]))
p.resetJointState(cabinet.object, 0, 0.1)

# load the robot
jointStartPositions = [0.0, 0.0, 0.0, -2*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.0, 0.0, 0.04, 0.04]
panda = Panda(basePosition=[0, 0, 0],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                jointStartPositions=jointStartPositions)

# main loop
offset1 = np.array([-0.15, 0.0, 0.1])       # reach above the cabinet
offset2 = np.array([-0.15, 0.0, 0.05])      # go down
offset3 = np.array([-0.3, 0.0, 0.05])       # pull the cabinet open
offset = [offset1, offset2, offset3]
timesteps = [401, 201, 401]
dataset = []
dataset_history = []
for idx in range(10):

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

    # perform the expert behavior
    for stage in range(3):
        for idx in range(1, timesteps[stage]):
            panda.move_to_pose(cabinet_position + offset[stage], ee_rotz=0, positionGain=0.01)
            p.stepSimulation()
            time.sleep(control_dt)
            if idx % 10 == 0:
                robot_state = panda.get_state()
                robot_pos = np.array(robot_state["ee-position"])
                action = robot_pos - prev_pos
                dataset.append(robot_pos.tolist() + cabinet_position.tolist() + action.tolist())
                robot_pos_history = robot_pos.tolist() + robot_positions[-1].tolist() +  robot_positions[-2].tolist()
                dataset_history.append(robot_pos_history + cabinet_position.tolist() + action.tolist())
                prev_pos = robot_pos
                robot_positions.append(np.copy(prev_pos))

# save the dataset of demonstrations
pickle.dump(dataset, open("HW9/datasets/dataset.pkl", "wb"))
pickle.dump(dataset_history, open("HW9/datasets/dataset_history.pkl", "wb"))
print("dataset has this many state-action pairs:", len(dataset))