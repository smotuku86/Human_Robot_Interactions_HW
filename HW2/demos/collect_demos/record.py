import pybullet as p
import pybullet_data
import numpy as np
import os
import time
import json
from robot import Panda
from teleop import KeyboardController
from cameras import ExternalCamera
import matplotlib.pyplot as plt
import argparse
#ok the idea here was to consolidate world setup, but the file only works if its inthe saem folder
#Future fix possible, but I just wanna get the demo recording working for now, so I'm leaving it as is.
from setup_world import setup_world, get_features_group, get_score, get_cube_positions

# parameters
control_dt = 1. / 240.

# setup the world
cubes, panda = setup_world()

# create a parser to set the number of the demonstration
parser = argparse.ArgumentParser()
parser.add_argument('--number', type=int, default=0, help='An integer input')
args = parser.parse_args()

# define save name and folders
savename = "demos/demo" + str(args.number) + ".json"
foldername = "demos/images/demo" + str(args.number) + "/"
os.makedirs(foldername, exist_ok=True)

# teleoperation interface
teleop = KeyboardController()

# camera for logging
external_camera = ExternalCamera(cameraDistance=1.0, 
                                    cameraYaw=40.0, 
                                    cameraPitch=-30.0,
                                    cameraTargetPosition=[0.5, 0.0, 0.2],
                                    cameraWidth=512, cameraHeight=512)

# get initial conditions
state = panda.get_state()
target_position = state["ee-position"]
target_quaternion = state['ee-quaternion']
cube_init_positions = get_cube_positions(cubes)
gripper_open = 1.0
theta_star = np.array([.5,0,1])

# main loop
demo = []
score = 0.0
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
        gripper_open = +1.0
    elif action[6] == -1:
        panda.close_gripper()
        gripper_open = -1.0

    # print when "." is pressed
    if action[7] == +1 and time.time() - last_press > 0.5:
        print("button pressed")
        # save image for logging
        image = external_camera.get_image()
        imagename = foldername + str(len(demo)) + '.png'
        plt.imsave(imagename, image)
        # collect features for logging
        state = panda.get_state()
        robot_position = state["ee-position"]
        cube_positions = get_cube_positions(cubes)
        features = get_features_group(robot_position, cube_init_positions, cube_positions)
        # get the human's score
        score = get_score(features, theta_star)
        # save the features and score
        demo.append(list(features))
        with open(savename, "w") as f:
            json.dump({"features": demo, "score": score}, f)
        last_press = time.time()

    # step simulation
    p.stepSimulation()
    time.sleep(control_dt)