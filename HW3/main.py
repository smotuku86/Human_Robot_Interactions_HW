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
    #offsets becasue box state returns centroid of box,
    # but we want to grab the top of the box
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

# Find the different between gripper orientation and possible goal orientation
def angle_diff(a, b):
    # returns smallest signed angle difference in [-pi, pi]
    return np.arctan2(np.sin(a - b), np.cos(a - b))

def predict_goal(initial_robot_state, current_robot_state, goals, beta = 1.0, rot_scale = 0.0):
    #predict what object the user is trying to get 
    #Returns a dict with the chance of each object being the goal
    #initial_robot_state and current_robot_state are both lists of length 4, with the first 3 values being the position and the last being rotation around z (euler)

    goal_predictions = {}

    #get and seperate the position and rotation values for the initial and current robot state
    initial_robot_state_position = np.array(initial_robot_state[0:3])
    initial_robot_state_rotz = initial_robot_state[3]
    current_robot_state_position = np.array(current_robot_state[0:3])
    current_robot_state_rotz = current_robot_state[3]

    for obj_name in goals.keys():
        #get the current state of selected goal/object
        obj_state_position = np.array(list(goals[obj_name]["position"]) )
        obj_state_rotz = goals[obj_name]["rotz"]

        #Prediction model - P(theta | robot location) 
        #for distance
        obj_distance_initial = np.linalg.norm(obj_state_position - initial_robot_state_position)
        obj_distance_current = np.linalg.norm(obj_state_position - current_robot_state_position)
        robot_distance_travelled = np.linalg.norm(current_robot_state_position - initial_robot_state_position)

        #for angle error
        obj_angle_error_current = abs(np.sin(angle_diff(obj_state_rotz, current_robot_state_rotz))) 
        #checks to see how parallel the robot is to the object - if they are parallel, this value will be 1, if they are perpendicular it will be 0
        #we want it to be perpendiculer so we can grasp it

        goal_predictions[obj_name] =  np.exp( beta * (obj_distance_initial - robot_distance_travelled - obj_distance_current) ) + rot_scale * np.exp( beta * obj_angle_error_current )
        #print(f"Raw prediction for {obj_name}: {goal_predictions[obj_name]:.4f} (Distance initial: {obj_distance_initial:.4f}, Distance current: {obj_distance_current:.4f}, Robot distance travelled: {robot_distance_travelled:.4f}, Angle error current: {obj_angle_error_current:.4f})")
        
    #normalize probabilities
    total = sum(goal_predictions.values())
    for obj_name in goal_predictions.keys():
        goal_predictions[obj_name] = goal_predictions[obj_name] / total
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

#for text
debug_text_id = None
robot_mode_id = None
autonomy_control_id = None
goal_prediction_id = None
#to see if user commanded robot movement
DidUserAct = False
#autonomy toggle to stop robot from moving on its own
autonomy_toggled = False
toggle_time = time.time()

# run simulation
# you can teleoperate the robot using the keyboard;
# see "teleop.py" for the mapping between keys and motions
state = panda.get_state()

init_position = state["ee-position"]
init_euler = state['ee-euler']
init_robot_state = list(init_position) + [init_euler[2]]

target_position = state["ee-position"]
target_quaternion = state['ee-quaternion']

#Code to initialize text for on screen

#goal prediction
goal_prediction_ids = {}
object_colors = {
    "box": [1, 0, 0],       # red
    "banana": [1, 1, 0],    # yellow
    "bottle": [0, 0, 1]     # blue
}
current_goal_texts = {obj: "" for obj in ["box", "banana", "bottle"]}
objects_dict = {"box": box, "banana": banana, "bottle": bottle}

for obj in ["box", "banana", "bottle"]:
    goal_prediction_ids[obj] = p.addUserDebugText(
        f"{obj}: 0.0",
        [0, 0, 0.8 - 0.05*["box","banana","bottle"].index(obj)], # stagger z so text doesn't overlap
        textColorRGB=object_colors[obj],
        textSize=1.2
    )
    current_goal_texts[obj] = f"{obj}: 0.0"


while True:

    #get current time
    current_time = time.time()

    #remove previous text if there is
    if robot_mode_id is not None:
        p.removeUserDebugItem(robot_mode_id)
    if autonomy_control_id is not None:
        p.removeUserDebugItem(autonomy_control_id)
    if debug_text_id is not None:
        p.removeUserDebugItem(debug_text_id)     
    if goal_prediction_id is not None:
        p.removeUserDebugItem(goal_prediction_id)



    # update the target pose
    action = teleop.get_action()
    #print("action: ", action)
    #check for user input
    DidUserAct = any(x != 0 for x in action)

    # this is where the human wants to go
    human_position = target_position + action[0:3]
    human_quaternion = p.multiplyTransforms([0, 0, 0], p.getQuaternionFromEuler(action[3:6]),
                                                [0, 0, 0], target_quaternion)[1]
    human_quaternion = np.array(human_quaternion)

    #get robot state to use for figuring out where the robot thinks it wants to go
    state = panda.get_state()
    robot_position = state["ee-position"]
    robot_quaternion = state['ee-quaternion']
    robot_euler = state['ee-euler']
    current_robot_state = list(robot_position) + [robot_euler[2]]
    
    # share autonomy between human and robot
    #Step 1: Predict which object the user is trying to get to
    goals = get_object_goals()
    #print("goals: ", goals)
    goal_predictions = predict_goal(init_robot_state, current_robot_state, goals, beta=15, rot_scale=0.0)
    #print("goal predictions: ", goal_predictions)

    #step 2: get robot action to reach each predicted goal
    object_actions = get_object_actions(robot_position, robot_euler, goals)
    #print("object actions: ", object_actions)

    #find highest probability goal and corresponding robot action
    best_goal = max(goal_predictions, key=goal_predictions.get)
    #print("best goal: ", best_goal)
    #choose our robot action
    robot_action = []
    if goal_predictions[best_goal] > 0.5: #only assist if we are more than 50% sure of the goal
        robot_action = object_actions[best_goal]

        debug_text_id = p.addUserDebugText( f"Assisting toward {best_goal}",
        [0, 0, 1],
        textColorRGB=[1,0,0],
        textSize=1.5)

    else: #stay in position - do not move
        robot_action = (robot_position, robot_quaternion) 

        debug_text_id = p.addUserDebugText( f"Goal Unsure",
        [0, 0, 1],
        textColorRGB=[1,0,0],
        textSize=1.5)

    #alpha tells us how much control the robot has vs the human 
    #I'm going to implement a policy that give robot control if human is not pressing any keys
    #and the human  control when they are pressing keys
    # this is simple but good becasue there is text that lets the user know what goal the robot
    #thinks the user wants. So, when the user sees the robot say it think it wants the goal they want, 
    # they can stop commanding movement and let the robot do the rest

    #toggle autonomy on/off when user presses "."
    if action[7] == +1 and current_time - toggle_time > 0.5:
        toggle_time = time.time() 
        autonomy_toggled = not autonomy_toggled

    #determine alpha based on autonomy and user input
    if autonomy_toggled:
        #when autonomy is on, give human control when they press keys, robot control otherwise
        if DidUserAct:
            alpha = 0.0
        else:
            alpha = 1.0
        autonomy_control_id = p.addUserDebugText( f"Autonomy On",
        [0, 0, 1.4],
        textColorRGB=[1,0,0],
        textSize=1.5)
    else:
        alpha = 0.0 #if autonomy is toggled off, human has full control
        autonomy_control_id = p.addUserDebugText( f"Autonomy Off",
        [0, 0, 1.4],
        textColorRGB=[1,0,0],
        textSize=1.5)

    #display robot control mode
    if DidUserAct:
        robot_mode_id = p.addUserDebugText( f"User Control",
        [0, 0, 1.2],
        textColorRGB=[1,0,0],
        textSize=1.5)
    else:
        robot_mode_id = p.addUserDebugText( f"Robot Full Control",
        [0, 0, 1.2],
        textColorRGB=[1,0,0],
        textSize=1.5)
        
        
   
   # blending human robot control
    target_position = (1-alpha) * human_position + alpha * np.array(robot_action[0])
    target_quaternion = (1-alpha) * human_quaternion + alpha * np.array(robot_action[1])

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

    #show goal predictions as text - update text each loop
    for obj_name, prob in goal_predictions.items():
        new_text = f"{obj_name}: {prob:.2f}"
        # only update if changed to avoid flashing
        if new_text != current_goal_texts[obj_name]:
            p.removeUserDebugItem(goal_prediction_ids[obj_name])
            goal_prediction_ids[obj_name] = p.addUserDebugText(
                new_text,
                objects_dict[obj_name].get_state()['position'] + np.array([0, 0, 0.1]), # position text above object
                textColorRGB=object_colors[obj_name],
                textSize=1.2
            )
            current_goal_texts[obj_name] = new_text
    # step simulation
    p.stepSimulation()
    time.sleep(control_dt)

    #chatlog - https://chatgpt.com/share/69994024-965c-8004-854d-57709bec1411 