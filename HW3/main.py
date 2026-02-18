import pybullet as p
import pybullet_data
import numpy as np
import os
import time
import json
from robot import Panda
from teleop import KeyboardController
from objects import objects



'''
Goal:
one or more videos of you teleoperating the robot to show the features of your program. This should include:
inferring the goal you want to reach
helping you reach that goal
stopping/starting assistance if you toggle shared autonomy on/off

to do - 
1. Add a function to predict the human's goal based on their position inputs.
2. Blend the human's teleoperation actions with the robot's assistive actions.
3. Adjust the blending factor `alpha` to find a good balance between assistance and human control. When the human is not pressing any keys, the robot should fully control its own motions.
4. Implement a toggle so that the robot stops/starts providing assistance if the human presses that key.
'''

# function that returns the goal positions and rotation around z for each object
# from our lecture notes, these are theta_box, theta_banana, etc.
def get_object_goals():
    box_state = box.get_state()
    banana_state = banana.get_state()
    bottle_state = bottle.get_state()
    #creae a dict with the names of each object - could pass a list in if the objects should not be hardcoded
    goals = {name: {} for name in ["box", "banana", "bottle"]}

    goals["box"]["position"] = box_state["position"] + np.array([0, 0, 0.05])
    goals["box"]["rotz"] = box_state["euler"][2] + np.pi/2
    goals["banana"]["position"] = banana_state["position"] + np.array([0, 0, -0.01])
    goals["banana"]["rotz"] = banana_state["euler"][2] + np.pi/2
    goals["bottle"]["position"] = bottle_state["position"] + np.array([-0.01, 0, 0.05])
    goals["bottle"]["rotz"] = bottle_state["euler"][2] + 0.0
    return goals

# function that outputs the actions to reach potential target
def get_object_actions(robot_position, robot_euler, goals):
    actions = {}
    actions["box"] = action_to_goal(robot_position, robot_euler, goals["box"]["position"], goals["box"]["rotz"])
    actions["banana"] = action_to_goal(robot_position, robot_euler, goals["banana"]["position"], goals["banana"]["rotz"])
    actions["bottle"] = action_to_goal(robot_position, robot_euler, goals["bottle"]["position"], goals["bottle"]["rotz"])
    return actions

# function that outputs the next target position and target quaternion if we are 
# reaching for the goal_position and goal_rotz
def action_to_goal(robot_position, robot_euler, goal_position, goal_rotz):
    position_error = goal_position - robot_position
    rotz_error = goal_rotz - robot_euler[2]
    if np.linalg.norm(position_error) > 0.01:
        position_error = position_error / np.linalg.norm(position_error)
    if np.abs(rotz_error) > 0.01:
        rotz_error = rotz_error / np.abs(rotz_error)
    # the gains 0.001 and 0.005 match the default pos_step and rot_step in teleop
    target_position = robot_position + 0.001 * position_error
    target_euler = np.array([np.pi, 0, robot_euler[2] + 0.005 * rotz_error])
    return target_position, np.array(p.getQuaternionFromEuler(target_euler))

def predict_goal(initial_robot_state, current_robot_state, goals, beta = 1.0):
    #predict what object the user is trying to get 
    #Returns a dict with the chance of each object being the goal
    #initial_robot_state and current_robot_state are both lists of length 4, with the first 3 values being the position and the last being rotation around z (euler)

    goal_predictions = {}

    initial_robot_state = np.array(initial_robot_state)
    current_robot_state = np.array(current_robot_state)

    for obj_name in goals.keys():
        obj_state = np.array(list(goals[obj_name]["position"]) + [goals[obj_name]["rotz"]])

        #Prediction model - P(theta | robot location)
        goal_predictions[obj_name] = np.exp( beta * (np.linalg.norm(obj_state - initial_robot_state)) ) / np.exp( beta*np.linalg.norm(current_robot_state-initial_robot_state) + beta*np.linalg.norm(obj_state - current_robot_state) )
    return goal_predictions


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
                                cameraPitch=-30.0, 
                                cameraTargetPosition=[0.5, 0.0, 0.2])

# load the objects
urdfRootPath = pybullet_data.getDataPath() # this only finds objects in the pybullet_data folder

plane = objects.PyBulletObject("plane.urdf", basePosition=[0, 0, -0.625])
table = objects.PyBulletObject("table/table.urdf", basePosition=[0.5, 0, -0.625])
box = objects.YCBObject("003_cracker_box.urdf", basePosition=[0.6, -0.2, 0.09], 
                                                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))
banana = objects.YCBObject("011_banana.urdf", basePosition=[0.7, 0.2, 0.025], 
                                                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))
bottle = objects.YCBObject("006_mustard_bottle.urdf", basePosition=[0.5, 0.05, 0.06], 
                                                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))

# load the robot
jointStartPositions = [0.0, 0.0, 0.0, -2*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.0, 0.0, 0.04, 0.04]
panda = Panda(basePosition=[0, 0, 0],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                jointStartPositions=jointStartPositions)

# teleoperation interface
teleop = KeyboardController()

# run simulation
# you can teleoperate the robot using the keyboard;
# see "teleop.py" for the mapping between keys and motions
state = panda.get_state()

init_position = state["ee-position"]
init_quaternion = state['ee-quaternion']
init_robot_state = list(init_position) + [init_quaternion[2]]

target_position = state["ee-position"]
target_quaternion = state['ee-quaternion']

while True:
    # update the target pose
    action = teleop.get_action()
    # this is where the human wants to go
    human_position = target_position + action[0:3]
    human_quaternion = p.multiplyTransforms([0, 0, 0], p.getQuaternionFromEuler(action[3:6]),
                                                [0, 0, 0], target_quaternion)[1]
    human_quaternion = np.array(human_quaternion)

    #get robot state to use for figuring out where the robot thinks it wants to go
    state = panda.get_state()
    robot_position = state["ee-position"]
    robot_quaternion = state['ee-quaternion']
    current_robot_state = list(robot_position) + [robot_quaternion[2]]


    #print("robot position: ", current_robot_state)
    
    # share autonomy between human and robot
    #Step 1: Predict which object the user is trying to get to
    goals = get_object_goals()
    #print("goals: ", goals)
    goal_predictions = predict_goal(init_robot_state, current_robot_state, goals)
    print("goal predictions: ", goal_predictions)

    ### to implement: currently we just execute human action ###
    # action = (1-alpha) * human_action + alpha * robot_action
    target_position = human_position
    target_quaternion = human_quaternion

    # impose workspace limit
    if target_position[2] < 0.02:
        target_position[2] = 0.02

    # move to the target pose
    panda.move_to_pose(ee_position=target_position, ee_quaternion=target_quaternion, positionGain=1)

    # open or close the gripper
    if action[6] == +1:
        panda.open_gripper()
    elif action[6] == -1:
        panda.close_gripper()

    # print when "." is pressed
    if action[7] == +1:
        print("button pressed")

    # step simulation
    p.stepSimulation()
    time.sleep(control_dt)