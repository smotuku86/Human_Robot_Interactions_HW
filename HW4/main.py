import pybullet as p
import pybullet_data
import numpy as np
import os
import time
from robot import Panda


# robot 1 will try to pick up the block numbered action1
# robot 2 will try to pick up the block numbered action2
# returns the score for each robot
def move_robots(action1, action2):
    # move to chosen blocks
    for idx in range(800):
        if action1 is -1:
            panda1.move_to_joint(positionGain=0.01)
        else:
            cube1 = get_cube_state(cubes, action1)
            panda1.move_to_pose(ee_position=cube1["position"], ee_rotz=cube1["euler"][2], positionGain=0.01)
        if action2 is -1:
            panda2.move_to_joint(positionGain=0.01)
        else:
            cube2 = get_cube_state(cubes, action2)
            panda2.move_to_pose(ee_position=cube2["position"], ee_rotz=cube2["euler"][2], positionGain=0.01)
        p.stepSimulation()
        time.sleep(control_dt) 
    # grasp blocks
    for idx in range(300):
        panda1.close_gripper()
        panda2.close_gripper()
        p.stepSimulation()
        time.sleep(control_dt)
    # return home
    for idx in range(800):
        panda1.move_to_joint(positionGain=0.01)
        panda2.move_to_joint(positionGain=0.01)
        p.stepSimulation()
        time.sleep(control_dt)
    # check for success
    cube1 = get_cube_state(cubes, action1)
    cube2 = get_cube_state(cubes, action2)
    score = [0, 0]
    if cube1["position"][2] > 0.3:
        score[0] = +1
    if cube2["position"][2] > 0.3:
        score[1] = +1
    # open grippers
    for idx in range(300):
        panda1.open_gripper()
        panda2.open_gripper()
        p.stepSimulation()
        time.sleep(control_dt)
    return score

# returns the position and orientation of a chosen cube
# cubes is the list of all cubes, and cube_number is the one you want
# cube_number is an integer between 0 and n_cubes - 1
def get_cube_state(cubes, cube_number):
    values = p.getBasePositionAndOrientation(cubes[cube_number])
    state = {}
    state["position"] = values[0]
    state["quaternion"] = values[1]
    state["euler"] = p.getEulerFromQuaternion(state["quaternion"])
    return state

# parameters
control_dt = 1. / 240.

# create simulation and place camera
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.resetDebugVisualizerCamera(cameraDistance=1.0, 
                                cameraYaw=0.0,
                                cameraPitch=-40.0, 
                                cameraTargetPosition=[0.0, 0.0, 0.2])

# load the objects
urdfRootPath = pybullet_data.getDataPath()
plane = p.loadURDF(os.path.join(urdfRootPath, "plane.urdf"), basePosition=[0, 0, -0.625])
table1 = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[-0.75, 0, -0.625])
table2 = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[+0.75, 0, -0.625])
# initialize n_cubes on the table
# their initial positions and orientations around the z axis are randomized
# cubes is a list of objects, each object is a cube
n_cubes = 5
cubes = []
for cube_number in range(n_cubes):
    cube_init_position = np.random.uniform([-0.15, -0.3, 0.025], [0.15, 0.3, 0.025], (3,))
    cube_init_angle = np.random.uniform([0, 0, 0], [0, 0, np.pi/2], (3,))
    cube = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), 
                            basePosition=cube_init_position,
                            baseOrientation=p.getQuaternionFromEuler(cube_init_angle))
    cubes.append(cube)

# load the robots
jointStartPositions = [0.0, 0.0, 0.0, -2*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.0, 0.0, 0.04, 0.04]
# load the first robot
panda1 = Panda(basePosition=[-0.7, 0, 0],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                jointStartPositions=jointStartPositions)
# load the second robot
panda2 = Panda(basePosition=[+0.7, 0, 0],
                baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi]),
                jointStartPositions=jointStartPositions)

# main loop
total_score = [0, 0]
for iteration in range(10):

    # choose which cube each robot should pick up
    # if the action is -1, the robot will sit out that round
    action1 = np.random.randint(0, n_cubes)
    action2 = np.random.randint(0, n_cubes)

    # robots try to grasp the chosen cubes and lift them
    iteration_score = move_robots(action1, action2)
    total_score[0] += iteration_score[0]
    total_score[1] += iteration_score[1]
    print("here is the score from that iteration: ", iteration_score)
    print("after", iteration+1, "rounds, the score is: ", total_score)

    # reset cubes to random positions for next round
    for cube in cubes:
        cube_init_position = np.random.uniform([-0.15, -0.3, 0.025], [0.15, 0.3, 0.025], (3,))
        cube_init_angle = np.random.uniform([0, 0, 0], [0, 0, np.pi/2], (3,))
        p.resetBasePositionAndOrientation(cube, cube_init_position, p.getQuaternionFromEuler(cube_init_angle))