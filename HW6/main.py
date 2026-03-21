import pybullet as p
import pybullet_data
import numpy as np
import os
import time
from robot import Panda
from objects import objects

#Helper Functions
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

def get_object_goals(cubes):
    goals = {}
    for cube in cubes:
        goals[cube]={}
        cube_state = get_cube_state(cubes, cube)
        goals[cube]["position"] = cube_state["position"] + np.array([0, 0, -0.01])
        goals[cube]["rotz"] = cube_state["euler"][2]
    return goals

# P(a | goal position)
# normalized across the two cubes
def human_model(action, robot_pos, cubes, chosen_cube, beta=1.0):
    #action: zeta, next planned move
    
    goal_probabilty = {}
    for cube in cubes:
        cube_state = get_cube_state(cubes, cube)
        cube_pos = cube_state["position"]
        curr_dist = np.linalg.norm(cube_pos - robot_pos)
        P = np.exp(beta * (np.linalg.norm(cube_pos - robot_pos + action) - curr_dist))
        goal_probabilty[cube] = P

    #normalize probabilities
    total = sum(goal_probabilty.values())
    for obj_name in goal_probabilty.keys():
        goal_probabilty[obj_name] = goal_probabilty[obj_name] / total

    P_chosen_cube = goal_probabilty[chosen_cube]
    return P_chosen_cube, goal_probabilty

def goal_probabilty(action, robot_pos, cubes, chosen_cube, Prior, beta=1.0):
    #Update the prior
    #might make a function
    P1 = Prior
    #print("P1",P1)

    prob_chosen_cube, prob_all_cubes = human_model(action, robot_pos, cubes, chosen_cube, beta)
    #print("prob cubes", prob_all_cubes)
    for PI_key in P1:
        P1[PI_key] = Prior[PI_key] * prob_all_cubes[PI_key]

    #normalize
    total = sum(P1.values())
    for obj_name in P1.keys():
        P1[obj_name] = P1[obj_name] / total
    return P1

# parameters
control_dt = 1. / 240.

# create simulation and place camera
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.resetDebugVisualizerCamera(cameraDistance=1.0, 
                                cameraYaw=90.0,
                                cameraPitch=-30.0, 
                                cameraTargetPosition=[0.5, 0.0, 0.2])

# load the objects
urdfRootPath = pybullet_data.getDataPath()
plane = p.loadURDF(os.path.join(urdfRootPath, "plane.urdf"), basePosition=[0, 0, -0.625])
table = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[0.5, 0, -0.625])
# load 3 cubes, randomly
n_cubes = 3
cubes = {}
'''
for cube_number in range(n_cubes):
    cube_init_position = np.random.uniform([.1, -0.3, 0.025], [0.6, 0.3, 0.025], (3,))
    cube_init_angle = np.random.uniform([0, 0, 0], [0, 0, np.pi/2], (3,))
    cube = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), 
                            basePosition=cube_init_position,
                            baseOrientation=p.getQuaternionFromEuler(cube_init_angle))
    cubes[cube_number] = cube
#print(cubes)
'''

y_spacing = 0.3 # Distance between cubes in the X direction

for cube_number in range(n_cubes):
    # Define base position
    # x starts at 0.1 and increases by x_spacing for each cube
    # y and z remain fixed
    cube_init_position = np.array([0.5, (cube_number * y_spacing) + -0.3, 0.025])
    
    # Define rotation angles (Euler)
    cube_init_angle = np.array([0, 0, 0]) # Cube_init_angle = np.array([0, 0, cube_init_angle[2]]) # Optional: vary rotation?
    
    # Create the URDF cube
    # Note: Ensure urdfRootPath is defined
    cube = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), 
                       basePosition=cube_init_position,
                       baseOrientation=p.getQuaternionFromEuler(cube_init_angle))
    
    # Add the cube to the list
    cubes[cube_number] = cube

# load the robot
jointStartPositions = [0.0, 0.0, 0.0, -2*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.0, 0.0, 0.04, 0.04]
panda = Panda(basePosition=[0, 0, 0],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                jointStartPositions=jointStartPositions)
 

# main loop
action_magnitude = 0.1

#initialize goal probabilty
intended_goal = 0
prob_goals = {}
for cube_number in cubes:
    prob_goals[cube_number] = 1/n_cubes

for idx in range(1000):

    # get the position of the robot and the cubes
    robot_state = panda.get_state()
    robot_pos = np.array(robot_state["ee-position"])

    #propose an action for the robot
    chosen_cube_pos = get_cube_state(cubes,intended_goal)["position"]
    n_samples = 100
    action_set = 1 * action_magnitude + (np.random.rand(n_samples,3) - .5)
    #print(action_set)
    best_legible_action = []
    best_score = 0
    for action in action_set:
        #print("action:", action)
        #find probabilty of goal being the chosen cube
        #given an action
        P1, prob_all_cubes = human_model(action, robot_pos, cubes, intended_goal, beta=1)
        if P1 > best_score:
            best_score = P1
            best_legible_action = action

    # the robot's action going straight to goal
    goal_seek_action = chosen_cube_pos - robot_pos
    if np.linalg.norm(goal_seek_action) > action_magnitude:
        goal_seek_action = action_magnitude * (goal_seek_action/ np.linalg.norm(goal_seek_action))

    if np.linalg.norm(best_legible_action) > action_magnitude:
        best_legible_action = action_magnitude * ( best_legible_action / np.linalg.norm(best_legible_action) )

    
    
    
  
    print("timestep:", idx, " human's belief over the cubes:", np.round(list(prob_goals.values()), 2))

    #blend legible action with goal seeking
    alpha = prob_goals[intended_goal]
    #print("goals", prob_goals)
    #print("alpha",alpha)

    #print("mag legible vs normal", np.linalg.norm(best_legible_action), " vs", np.linalg.norm(goal_seek_action))

    robot_action = (1-alpha) * best_legible_action + alpha * goal_seek_action

    # update human estimate of the goal
    prob_goals = goal_probabilty(robot_action, robot_pos, cubes, intended_goal, prob_goals, beta=.25)
    # take the action
    panda.move_to_pose(robot_pos + robot_action, ee_rotz=0, positionGain=0.01)

    p.stepSimulation()
    time.sleep(control_dt)

    #make blending factor as how confident the human is of goal
