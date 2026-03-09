import pybullet as p
import pybullet_data
import numpy as np
import os
import time
from robot import Panda
from objects import objects
from openai import OpenAI
from secret_key import sk
from teleop import KeyboardController
import json
import re

#for web server
import requests

# now you can read from task_queue in your PyBullet loop
def get_env_state():
    
    robot_state = panda.get_state()
    cube1_state = cube1.get_state()
    cube2_state = cube2.get_state()
    cube3_state = cube3.get_state()
    cabinet_state = cabinet.get_state()
    microwave_state = microwave.get_state()

    #mod the handle states a little to let the robot grab it 
    microwavehandle_euler = np.array(p.getEulerFromQuaternion(microwave_state["handle_quaternion"]))
    microwavehandle_euler_rotz = np.array([0,0,np.pi])
    microwave_state["handle_quaternion"] = np.array(p.getQuaternionFromEuler(microwavehandle_euler + microwavehandle_euler_rotz))

    cabinet_handle_euler = np.array(p.getEulerFromQuaternion(cabinet_state["handle_quaternion"]))
    cabinet_handle_euler_rotz = np.array([0,0,np.pi])
    cabinet_state["handle_quaternion"] = np.array(p.getQuaternionFromEuler(cabinet_handle_euler + cabinet_handle_euler_rotz))

    env_state = {
        "robot": {
            "ee_position": robot_state["ee-position"],
            "ee_quaternion": robot_state["ee-quaternion"]
        },

        "cubes": {
            "cube1": {
                "position": cube1_state["position"],
                "quaternion": cube1_state["quaternion"]
            },
            "cube2": {
                "position": cube2_state["position"],
                "quaternion": cube2_state["quaternion"]
            },
            "cube3": {
                "position": cube3_state["position"],
                "quaternion": cube3_state["quaternion"]
            }
        },

        "cabinet": {
            "position": cabinet_state["base_position"],
            "quaternion": cabinet_state["base_quaternion"],
            "handle_position": cabinet_state["handle_position"],
            "handle_quaternion": cabinet_state["handle_quaternion"],
            "joint_angle": cabinet_state["joint_angle"]
        },

        "microwave": {
            "position": microwave_state["base_position"],
            "quaternion": microwave_state["base_quaternion"],
            "handle_position": microwave_state["handle_position"],
            "handle_quaternion": microwave_state["handle_quaternion"],
            "joint_angle": microwave_state["joint_angle"]
    }
}
    return env_state

import numpy as np

#helper to scoring function to check if cube is in box
def is_inside(cube_pos, obj_name, obj_pos, obj_yaw=0.0):
    """
    Check if a cube (point) is inside a cabinet or microwave.
    
    cube_pos  : (x, y, z) world position of the cube
    obj_name  : "cabinet" or "microwave"
    obj_pos   : (x, y, z) world position of the object origin
    obj_yaw   : rotation around Z axis in radians (default 0)
    """

    bounds = {
        "microwave": {
            "x": (-0.10, +0.10),
            "y": (-0.15, +0.15),
            "z": (-0.10, +0.10),
        },
        "cabinet": {
            "x": (-0.10, +0.10),
            "y": (-0.15, +0.15),
            "z": (-0.10, +0.10),
        },
    }

    if obj_name not in bounds:
        raise ValueError(f"Unknown object: {obj_name}. Use 'cabinet' or 'microwave'.")

    # Translate cube into object's local frame
    delta = np.array(cube_pos) - np.array(obj_pos)

    # Yaw rotation matrix (around Z, undoing object's yaw)
    rot = np.array([
        [ np.cos(-obj_yaw), -np.sin(-obj_yaw), 0],
        [ np.sin(-obj_yaw),  np.cos(-obj_yaw), 0],
        [0,                  0,                 1],
    ])

    #this is a vector of the displacement between 
    #the cube and orgin of a box, accounting for the box's rotation
    local = rot @ delta

    b = bounds[obj_name]
    is_it_inside = ( ( b["x"][0] <= local[0] <= b["x"][1] ) and
                     ( b["y"][0] <= local[1] <= b["y"][1] ) and
                     ( b["z"][0] <= local[2] <= b["z"][1] ) )
    return is_it_inside

scoring_state = {
    "cabinet": {"is_open": False, "open_count": 0, "close_count": 0, "scored": False},
    "microwave": {"is_open": False, "open_count": 0, "close_count": 0}, "scored": False,
    "cube1": {"in_box": False},
    "cube2": {"in_box": False},
    "cube3": {"in_box": False},
}

def scoring_state_machine(score, scoring_state):
    #funtion run at end of loop to tell us the current score 
    '''
    microwave size from origin:
    X: +0.10 to +0.10
    Y: -0.15 to +0.15 
    Z: -0.10 to +0.10
    Cabinet is same size
    '''
    #Get states of everything we need
    env_state = get_env_state()

    cube1_pos = env_state['cubes']['cube1']["position"]
    cube2_pos = env_state['cubes']['cube2']["position"]
    cube3_pos = env_state['cubes']['cube3']["position"]

    microwave_origin = env_state['microwave']['position']
    microwave_zrot = p.getEulerFromQuaternion(env_state['microwave']['quaternion'])[2]
    microwave_handle_origin = env_state['microwave']['handle_position']
    microwave_hinge_angle = env_state['microwave']['joint_angle']
    cabinet_origin = env_state['cabinet']['position']
    cabinet_zrot = p.getEulerFromQuaternion(env_state['cabinet']['quaternion'])[2]
    cabinet_handle_origin = env_state['cabinet']['handle_position']
    cabinet_drawer_displacement = env_state['cabinet']['joint_angle']

    #hardcoded number of cubes
    cube_positions = [cube1_pos, cube2_pos, cube3_pos]

    for i, cube_pos in enumerate(cube_positions):
        obj_name = f"cube{i+1}"
        #if the cube is not in any boxes - check for it
        if not scoring_state[obj_name]["in_box"]:
            in_cabinet = is_inside(cube_pos, "cabinet", cabinet_origin, cabinet_zrot) 
            in_microwave = is_inside(cube_pos, "microwave", microwave_origin, microwave_zrot)
            scoring_state[obj_name]["in_box"] = in_cabinet or in_microwave

            #if cube got put in box, add 1 point
            if scoring_state[obj_name]["in_box"]:
                score +=1

    #check for if the micro wave got opened and closed 
    # +1 point if so (only on first time)
    if (microwave_hinge_angle > 45 and not scoring_state["microwave"]["is_open"]):
        scoring_state["microwave"]["is_open"] = True
        scoring_state["microwave"]["open_count"] += 1
    elif (scoring_state["microwave"]["is_open"] and microwave_hinge_angle < 3):
        scoring_state["microwave"]["is_open"] = False
        scoring_state["microwave"]["close_count"] += 1

    if (scoring_state["microwave"]["open_count"] == 1 and
        scoring_state["microwave"]["close_count"] == 1 and
        not scoring_state["microwave"]["scored"]):

        scoring_state["microwave"]["scored"] = True
        score += 1

    #check for if the cabinet got opened and closed 
    # +1 point if so (only on first time)
    if (cabinet_drawer_displacement > .1 and not scoring_state["cabinet"]["is_open"]):
        scoring_state["cabinet"]["is_open"] = True
        scoring_state["cabinet"]["open_count"] += 1
    elif (scoring_state["cabinet"]["is_open"] and cabinet_drawer_displacement < .02):
        scoring_state["cabinet"]["is_open"] = False
        scoring_state["cabinet"]["close_count"] += 1

    if (scoring_state["cabinet"]["open_count"] == 1 and
        scoring_state["cabinet"]["close_count"] == 1 and
        not scoring_state["cabinet"]["scored"]):

        scoring_state["cabinet"]["scored"] = True
        score += 1

    return score, scoring_state

#chat lowkey cooked this one up
def clean_json_block(text: str) -> str:
    """
    Removes ```json or ``` and trailing ``` from a string containing JSON.
    Returns the clean JSON string.
    """
    # Remove ```json at the start 
    text = re.sub(r"^```(?:json)?\s*", "", text)
    # Remove ``` at the end
    text = re.sub(r"\s*```$", "", text)
    return text.strip()

# function that outputs the next target position and target quaternion if we are 
# reaching for the goal_position and goal_rotz
def action_to_goal(robot_position, robot_quat, goal_position, goal_quat):
    position_error = np.array(goal_position) - np.array(robot_position)
    robot_euler = np.array(p.getEulerFromQuaternion(robot_quat))
    goal_euler = np.array(p.getEulerFromQuaternion(goal_quat)) 
    rotx_error = goal_euler[0] - robot_euler[0]
    roty_error = goal_euler[1] - robot_euler[1]
    rotz_error = goal_euler[2] - robot_euler[2]
    if np.linalg.norm(position_error) > 0.01:
        position_error = position_error / np.linalg.norm(position_error)
    if np.abs(rotx_error) > 0.01:
        rotx_error = rotx_error / np.abs(rotx_error)
    if np.abs(roty_error) > 0.01:
        roty_error = roty_error / np.abs(roty_error)
    if np.abs(rotz_error) > 0.01:
        rotz_error = rotz_error / np.abs(rotz_error)
    
    # the gains 0.001 and 0.005 match the default pos_step and rot_step in teleop
    target_position = robot_position + 0.001 * position_error
    rot_scale = .005
    target_rotx = robot_euler[0] + rot_scale * rotx_error
    target_roty = robot_euler[1] + rot_scale * roty_error
    target_rotz = robot_euler[2] + rot_scale * rotz_error

    target_euler = np.array([robot_euler[0],robot_euler[1], target_rotz])
    return target_position, np.array(p.getQuaternionFromEuler(target_euler))

''' Not using rn
#setup connection to local langauge model
# Ollama runs locally and exposes an OpenAI-compatible API
client = OpenAI(
    api_key="ollama",  # required but ignored
    base_url="http://127.0.0.1:11434/v1"
)
model = "qwen2.5-coder:7b"
'''

# Modify OpenAI's API key and API base to use the server.
openai_api_key = sk
openai_api_base = "https://llm-api.arc.vt.edu/api/v1"

client = OpenAI(
    api_key=openai_api_key,
    base_url=openai_api_base,
)
model = "gpt-oss-120b"

#read in Ai Prompt to give to LLM
# Read a local file
here = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(here, "AI_prompt.txt"), "r", encoding="utf-8") as f:
    file_content = f.read()

# parameters
start_time = time.time()  # start time
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

# # load the objects
urdfRootPath = pybullet_data.getDataPath() 
plane = p.loadURDF(os.path.join(urdfRootPath, "plane.urdf"), basePosition=[0, 0, -0.625])
table = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[0.5, 0, -0.625])
cube1 = objects.SimpleObject("cube.urdf", basePosition=[0.5, -0.3, 0.025], baseOrientation=p.getQuaternionFromEuler([np.pi, 0, 0.7]))
cube2 = objects.SimpleObject("cube.urdf", basePosition=[0.4, -0.2, 0.025], baseOrientation=p.getQuaternionFromEuler([np.pi, 0, -0.3]))
cube3 = objects.SimpleObject("cube.urdf", basePosition=[0.5, -0.1, 0.025], baseOrientation=p.getQuaternionFromEuler([np.pi, 0, 0.2]))
cabinet = objects.CollabObject("cabinet.urdf", basePosition=[0.9, -0.3, 0.2], baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi]))
microwave = objects.CollabObject("microwave.urdf", basePosition=[0.5, 0.3, 0.2], baseOrientation=p.getQuaternionFromEuler([0, 0, -np.pi/2]))

# load the robot
#jointStartPositions = [0.0, 0.0, 0.0, -2*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.0, 0.0, 0.04, 0.04]
jointStartPositions = [-0.3015089621607984, -0.97795221389853, 0.22442506269298454, -2.7081846651776735, 2.4854378663851446, 2.7808122637773205, 1.6185771651945327, 0.0, 0.0, 0.06, 0.06]
panda = Panda(basePosition=[0, 0, 0],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                jointStartPositions=jointStartPositions)

# teleoperation interface
teleop = KeyboardController()

#init variables for teleop
state = panda.get_state()
target_position = state["ee-position"]
target_quaternion = state['ee-quaternion']
#z limit to not crash into table
z_buffer = .02

#init variables for robot action
#so it starts as it reached the goal  - and so it asks for a prompt
robot_action = [target_position, target_quaternion]
plan = {"target_position" : target_position, 
        "target_quaternion": target_quaternion }
goal_reached = True

#so the script doesnt spam us with input requests
waiting_for_input = False

#print(get_env_state())

#flip this true/false for autonomy to assist
AssistanceOn = True


#vaibles to keep track of score 
score = 0

# main loop
while True:

    #get current time
    current_time = time.time()

    #get user action
    action = teleop.get_action()
    #check for user input
    DidUserAct = any(x != 0 for x in action)

    # this is where the human wants to go
    human_position = target_position + action[0:3]
    human_quaternion = p.multiplyTransforms([0, 0, 0], p.getQuaternionFromEuler(action[3:6]),
                                                [0, 0, 0], target_quaternion)[1]
    human_quaternion = np.array(human_quaternion)
    human_euler = p.getEulerFromQuaternion(human_quaternion)

    # example how how you can get information about objects
    # try printing these states to see what they contain
    robot_state = panda.get_state()
    cube1_state = cube1.get_state()
    cube2_state = cube2.get_state()
    cube3_state = cube3.get_state()
    cabinet_state = cabinet.get_state()
    microwave_state = microwave.get_state()

    # here the robot action will be chosen by the llm

    #first check if the action is still on going
    # if the goal of teh robot action is reached, then prompt a new action
    LLM_response = {}
    response = requests.get("http://localhost:8000/next_task")
    task = response.json().get("task")  # task could be None
    

    if (task != None) and (np.linalg.norm(np.array(robot_state['ee-position']) - np.array(robot_action[0])) < .01):
        # prompt a new action
        goal_reached = False

        prompt = f'''{file_content} 
                Environment State: {get_env_state()}
                {task}'''
        response = client.chat.completions.create(
            model=model,
            messages=[
                {"role": "system", "content": "You are a helpful robotics assistant."},
                {"role": "user", "content": prompt}
            ]
        )

        LLM_response = response.choices[0].message.content
        # parse the response to get the robot action
        # convert JSON string → Python dict
        plan = json.loads(LLM_response)
        LLM_response = clean_json_block(LLM_response)
        requests.post("http://localhost:8000/llm_response",
            json=plan)
        # get target position and quaternion
        robot_action = action_to_goal(robot_state["ee-position"], robot_state['ee-quaternion'],plan["target_position"], plan["target_quaternion"])
        robot_intended_action = plan["action"]
        target_object = plan["target_object"]

        print("Robot target position:", robot_action[0])
        print("Robot target quaternion:", robot_action[1])
        print("Action:", robot_intended_action)
        print("Target object:", target_object)
    else:
        #keep current robot actions goign unless it has reached the goal
        if np.linalg.norm(np.array(robot_state['ee-position']) - np.array(plan["target_position"])) > .01:
            robot_action = action_to_goal(robot_state["ee-position"], robot_state['ee-quaternion'],plan["target_position"], plan["target_quaternion"])
        else:
            goal_reached = True
            goal_reached_message = {"Action": "Nothing"}
            requests.post("http://localhost:8000/llm_response",
            json=goal_reached_message)
        
    
    alpha = 0 #default no assist

    if AssistanceOn:
        alpha = .8

        if DidUserAct: 
            #if the user is commadning during assistance, turn assist down
            alpha = 0.1
        if goal_reached:
            #if the goal was reached, turn assistance off
            #robot is will continue helping after a new prompt is given
            alpha = 0
    
    # blending human robot control
    target_position = (1-alpha) * human_position + alpha * np.array(robot_action[0])
    robot_euler = p.getEulerFromQuaternion(robot_action[1])
    blended_euler = (1-alpha) * np.array(human_euler) + alpha * np.array(robot_euler)
    target_quaternion = p.getQuaternionFromEuler(blended_euler)

    #Check for table interference
    if target_position[2] + action[2] < z_buffer:
        action[2] = 0
    
    # move to the target pose
    panda.move_to_pose(ee_position=target_position, ee_quaternion=target_quaternion)

    # open or close the gripper
    if action[6] == +1:
        panda.open_gripper()
        gripper_open = True
    elif action[6] == -1:
        panda.close_gripper()
        gripper_open = False

    # print state
    if action[7] == +1:
        print(robot_state["joint-position"])
    
    #update score
    score, scoring_state = scoring_state_machine(score, scoring_state)

    timer = current_time - start_time

    requests.post("http://localhost:8000/update_score", json={"score": score})
    requests.post("http://localhost:8000/update_time", json={"time": round(timer, 2)})
    
    # step the simulation
    p.stepSimulation()
    time.sleep(control_dt)