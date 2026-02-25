import pybullet as p
import pybullet_data
import numpy as np
import os
import time
from robot import Panda

'''
2. You can only control one robot. The other robot chooses blocks uniformly at random.
 Develop a strategy to maximize the overall success of the team, and then implement it. 
 You may need to try multiple strategies, and then compare them.
 '''

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

def get_cube_distances(cubes, robot):
    # cubes is a dict of all cubes
    # robot is the robot object we want to find the distances from

    #returns a dict of the distances of each cube from the robot
    robot_state = robot.get_state()
    cube_distances = {}
    
    for cube_number in cubes:
        cube_state = get_cube_state(cubes, cube_number)
        cube_position = cube_state["position"]
        robot_position = robot_state["ee-position"]
        distance = np.linalg.norm(np.array(cube_position) - np.array(robot_position))
        cube_distances[cube_number] = distance
    return cube_distances
    
# parameters
control_dt = 1. / 240.

# create simulation and place camera
physicsClient = p.connect(p.DIRECT) #no gui, run faster do p.DIRECT
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
cubes = {}
for cube_number in range(n_cubes):
    cube_init_position = np.random.uniform([-0.15, -0.3, 0.025], [0.15, 0.3, 0.025], (3,))
    cube_init_angle = np.random.uniform([0, 0, 0], [0, 0, np.pi/2], (3,))
    cube = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), 
                            basePosition=cube_init_position,
                            baseOrientation=p.getQuaternionFromEuler(cube_init_angle))
    cubes[cube_number] = cube


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

'''
1.You can control both robots. Develop a strategy for the robots to efficiently 
pick up the blocks without collisions. Describe your strategy in words or pseudocode, 
and then implement it.

Find the closest cube to each robot, and assign that cube to that robot. 

If the cubes are too close to each other, make one robot go first, and command the other
robot to wait until the first robot is done, or find the next closest cube for the second robot
(runs the same promixity check)

So the pseudocode for this strategy is:

Get the state of environment - cubes and robots
find the distances of the cubes from each robot
assign the closest cube to each robot
run a proximity check 
reassign cubes if they are too close
go get the cubes

'''

total_score = [0, 0]
for iteration in range(10):

    #this is scuffed we assume we know what the random choice is - fix
    
    # choose which cube each robot should pick up
    # robot 2 is random
    r2_chosen_cube = np.random.randint(0, n_cubes)

    #find the cube robot 1 should find
    #finds how close each cube is to the robots
    c2r1_distances = get_cube_distances(cubes, panda1)

    #find the closest cube to each robot
    #but also sort them incase the two robots are closest to the same cube
    r1_sorted_cubes = sorted(c2r1_distances, key=c2r1_distances.get)
        #key makes sure the min function compares values of dict, not keys
        #chosen cube should the key of cubes 
    #Run a proximity check to see if the cubes are too close to each other
    
    while abs(r1_sorted_cubes[0] - r2_chosen_cube) < .1:
        #if they are too close, assign one of the robots the next closest cube
        r1_sorted_cubes = r1_sorted_cubes[1:]

        if len(r1_sorted_cubes) == 0:
            #means there are no other cubes
            #tell it to wait
            r1_sorted_cubes = [-1]
            break
    
    r1_chosen_cube = r1_sorted_cubes[0]
    #now our cube is chosen

    # if the action is -1, the robot will sit out that round
    action1 = r1_chosen_cube
    action2 = r2_chosen_cube

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
        p.resetBasePositionAndOrientation(cubes[cube], cube_init_position, p.getQuaternionFromEuler(cube_init_angle))