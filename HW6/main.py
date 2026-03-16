import pybullet as p
import pybullet_data
import numpy as np
import os
import time
from robot import Panda
from objects import objects


# P(a | goal position)
# normalized across the two cubes
def human_model(action, robot_pos, cube1_pos, cube2_pos, beta=1.0):
    curr_dist1 = np.linalg.norm(cube1_pos - robot_pos)
    curr_dist2 = np.linalg.norm(cube2_pos - robot_pos)
    P1 = np.exp(beta * (np.linalg.norm(cube1_pos - robot_pos + action) - curr_dist1))
    P2 = np.exp(beta * (np.linalg.norm(cube2_pos - robot_pos + action) - curr_dist2))
    return P1 / (P1 + P2)

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
cube1 = objects.SimpleObject("cube.urdf", basePosition=[0.5, -0.3, 0.025])
cube2 = objects.SimpleObject("cube.urdf", basePosition=[0.6, -0.2, 0.025])

# load the robot
jointStartPositions = [0.0, 0.0, 0.0, -2*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.0, 0.0, 0.04, 0.04]
panda = Panda(basePosition=[0, 0, 0],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                jointStartPositions=jointStartPositions)


for idx in range(5000):
    p.stepSimulation()
    time.sleep(control_dt)    

# main loop
action_magnitude = 0.1
P = np.array([0.5, 0.5])
for idx in range(500):

    # get the position of the robot and the cubes
    robot_state = panda.get_state()
    cube1_state = cube1.get_state()
    cube2_state = cube2.get_state()
    robot_pos = np.array(robot_state["ee-position"])
    cube1_pos = np.array(cube1_state["position"])
    cube2_pos = np.array(cube2_state["position"])

    # select the robot's action
    action = cube1_pos - robot_pos
    if np.linalg.norm(action) > action_magnitude:
        action *= action_magnitude / np.linalg.norm(action)

    # update human estimate of the goal
    prob_cube1 = human_model(action, robot_pos, cube1_pos, cube2_pos, beta=0.25)
    P[0] *= prob_cube1
    P[1] *= (1 - prob_cube1)
    P /= np.sum(P)
    print("timestep:", idx, " human's belief over the cubes:", np.round(P, 2))

    # take the action
    panda.move_to_pose(robot_pos + action, ee_rotz=0, positionGain=0.01)

    p.stepSimulation()
    time.sleep(control_dt)
