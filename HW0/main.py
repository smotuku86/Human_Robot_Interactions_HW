import pybullet as p
import pybullet_data
import numpy as np
import os
import time
from robot import Panda


# parameters
control_dt = 1. / 240.

#Create Functions 
def move_to_pose(ee_position, ee_rotz):
    for i in range (800):
        panda.move_to_pose(ee_position=ee_position, ee_rotz=ee_rotz, positionGain=0.01)
        p.stepSimulation()
        time.sleep(control_dt)  
    return

def close_gripper():
    for i in range (300):
        panda.close_gripper()
        p.stepSimulation()
        time.sleep(control_dt)
    return

def open_gripper():
    for i in range (300):
        panda.open_gripper()
        p.stepSimulation()
        time.sleep(control_dt)
    return

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
cube = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), basePosition=[0.6, -0.2, 0.05])

# load the robot
jointStartPositions = [0.0, 0.0, 0.0, -2*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.0, 0.0, 0.04, 0.04]
panda = Panda(basePosition=[0, 0, 0],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                jointStartPositions=jointStartPositions)

# let the scene initialize
for i in range (200):
    p.stepSimulation()
    time.sleep(control_dt)


#LM Output
# run sequence of position and gripper commands