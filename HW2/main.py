import pybullet as p
import pybullet_data
import numpy as np
import os
import time
import json
from robot import Panda
from teleop import KeyboardController
from setup_world import setup_world, get_features_group, get_score, get_cube_positions

# parameters
control_dt = 1. / 240.

# setup the world
cubes, panda = setup_world()

# teleoperation interface
teleop = KeyboardController()

# get initial conditions
state = panda.get_state()
target_position = state["ee-position"]
target_quaternion = state['ee-quaternion']

cube_init_position = get_cube_positions(cubes)

# set your scoring function parameters
# values can be between -1 and +1
# the score is transpose(feature) * theta
# theta = [-1.0, 0, 0, 0] - old
theta = [0.99049298, 0.14766348, 0.31938759] #got from Metropolis Hastings - optimized theta from learn_theta.py 
theta = [1, 0.46424706, 0.0973374]

# main loop
last_press = time.time()
while True:
    # update the target pose
    action = teleop.get_action()
    target_position = target_position + action[0:3]
    target_quaternion = p.multiplyTransforms([0, 0, 0], p.getQuaternionFromEuler(action[3:6]),
                                                [0, 0, 0], target_quaternion)[1]
    # move to the target pose
    panda.move_to_pose(ee_position=target_position, ee_quaternion=target_quaternion)

    # open or close the gripper
    if action[6] == +1:
        panda.open_gripper()
    elif action[6] == -1:
        panda.close_gripper()

    # print when "." is pressed
    if action[7] == +1:
        state = panda.get_state()
        robot_position = state["ee-position"]
        cube_position = get_cube_positions(cubes)
        feature = get_features_group(robot_position, cube_init_position, cube_position)
        score = get_score(feature, theta)
        print("features are:", np.round(feature, 3))
        print("score is:", np.round(score, 3))

    # step simulation
    p.stepSimulation()
    time.sleep(control_dt)