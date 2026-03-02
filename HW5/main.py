import pybullet as p
import pybullet_data
import numpy as np
import os
import time
from robot import Panda
from objects import objects


# parameters
control_dt = 1. / 240.

# create simulation and place camera
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.resetDebugVisualizerCamera(cameraDistance=1.0, 
                                cameraYaw=40.0,
                                cameraPitch=-40.0, 
                                cameraTargetPosition=[0.5, 0.0, 0.2])

# load the objects
urdfRootPath = pybullet_data.getDataPath()
plane = p.loadURDF(os.path.join(urdfRootPath, "plane.urdf"), basePosition=[0, 0, -0.625])
table = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[0.5, 0, -0.625])
cube1 = objects.SimpleObject("cube.urdf", basePosition=[0.5, -0.3, 0.025], baseOrientation=p.getQuaternionFromEuler([0, 0, 0.7]))
cube2 = objects.SimpleObject("cube.urdf", basePosition=[0.4, -0.2, 0.025], baseOrientation=p.getQuaternionFromEuler([0, 0, -0.3]))
cube3 = objects.SimpleObject("cube.urdf", basePosition=[0.5, -0.1, 0.025], baseOrientation=p.getQuaternionFromEuler([0, 0, 0.2]))
cabinet = objects.CollabObject("cabinet.urdf", basePosition=[0.9, -0.3, 0.2], baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi]))
microwave = objects.CollabObject("microwave.urdf", basePosition=[0.5, 0.3, 0.2], baseOrientation=p.getQuaternionFromEuler([0, 0, -np.pi/2]))

# load the robot
jointStartPositions = [0.0, 0.0, 0.0, -2*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.0, 0.0, 0.04, 0.04]
panda = Panda(basePosition=[0, 0, 0],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                jointStartPositions=jointStartPositions)

# main loop
while True:

    # example how how you can get information about objects
    # try printing these states to see what they contain
    robot_state = panda.get_state()
    cube1_state = cube1.get_state()
    cabinet_state = cabinet.get_state()

    # step the simulation
    p.stepSimulation()
    time.sleep(control_dt)