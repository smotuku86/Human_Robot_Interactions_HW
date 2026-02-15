import pybullet as p
import pybullet_data
import numpy as np
import os
import time
import json
from robot import Panda

# define the relevant features
# features are (1) robot distance from cube and
# (2-4) cube distance from initial position in x, y, and z
def get_features(robot_position, cube_position, cube_init_position):
    robot_position = np.array(robot_position)
    cube_position = np.array(cube_position)
    cube_init_position = np.array(cube_init_position)
    feature = np.array([np.linalg.norm(robot_position - cube_position), 
                                abs(cube_position[0] - cube_init_position[0]),
                                abs(cube_position[1] - cube_init_position[1]),
                                abs(cube_position[2] - cube_init_position[2])])
    return feature

# New Feature Function to score grouping cubes together
#feature 1 - how claose are the cubes
#feature 2 - is the robot close to the cubes
#feature 3 - how high are the cubes on avg (we might want to stack them)
def get_features_group(robot_position,cube_init_positions, cube_positions):
    cube_positions = np.array(cube_positions)
    cube_init_positions = np.array(cube_init_positions)
    # find centeroid of the cubes
    # we want to minimize the distance between the cubes and the centeroid
    centeroid_x = np.mean(cube_positions[:,0])
    centeroid_y = np.mean(cube_positions[:,1])
    centeroid_z = np.mean(cube_positions[:,2])
    # get each cube's distance from the centeroid and take the mean - we want to minimize this distance
    cube_distances = np.linalg.norm(cube_positions - np.array([centeroid_x, centeroid_y, centeroid_z]), axis=1)
    # taking inverse bc get_score multiplies feature by theta, 
    # so increased score relates to minimizing distance between cubes and centeroid
    feature1 = 1/np.mean(cube_distances)

    # additional features:
    # like robot distance from centeroid, in x,y,z
    feature2  = np.linalg.norm(robot_position - np.array([centeroid_x, centeroid_y, centeroid_z]))
    # and avg height of the cubes - we would want to make this higher to stack the cubes
    #tbh not the best way to get stack height
    feature3 = centeroid_z
    return np.array([feature1, feature2, feature3])

# get the score for your choice of theta
def get_score(feature, theta):
    theta = np.array(theta)
    scale = np.array([.2, 1, 10]) # need to scale feature 1 down as it was not modeled that great in the beginning 
                                  # and feature 3 as it is too small compared to the other features, so it does not contribute much to the score.
    feature = np.array(feature)
    return (feature * scale) @ theta

# randomize the cube x-y location
def random_cube_location():
    cube_x = np.random.uniform(0.4, 0.6)
    cube_y = np.random.uniform(-0.3, +0.3)
    return cube_x, cube_y

#function to get list of cube positions
def get_cube_positions(cubes):
    cube_positions = []
    for cube_num in range(1, len(cubes) + 1):
        cube_positions.append(p.getBasePositionAndOrientation(cubes[f"cube{cube_num}"])[0])
    return cube_positions


#function to set_up env
def setup_world():
    # create simulation and place camera
    physicsClient = p.connect(p.GUI)
    p.setGravity(0, 0, -9.81)
    # disable keyboard shortcuts so they do not interfere with keyboard control
    p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(cameraDistance=1.0, 
                                    cameraYaw=40.0,
                                    cameraPitch=-30.0, 
                                    cameraTargetPosition=[0.5, 0.0, 0.2])

    # load the objects
    urdfRootPath = pybullet_data.getDataPath()
    plane = p.loadURDF(os.path.join(urdfRootPath, "plane.urdf"), basePosition=[0, 0, -0.625])
    table = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[0.5, 0, -0.625])

    numCubes = 3
    cubes = {} # dictionary to hold cube ids
    for cube_num in range(1, numCubes + 1):
        cube_x, cube_y = random_cube_location()
        cubes[f"cube{cube_num}"] = p.loadURDF(
            os.path.join(urdfRootPath, "cube_small.urdf"),
            basePosition=[cube_x, cube_y, 0.025]
        )

    # load the robot
    jointStartPositions = [0.0, 0.0, 0.0, -2*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.0, 0.0, 0.04, 0.04]
    panda = Panda(basePosition=[0, 0, 0],
                    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                    jointStartPositions=jointStartPositions)
    return cubes, panda