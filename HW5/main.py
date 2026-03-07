import pybullet as p
import pybullet_data
import numpy as np
import os
import time
from robot import Panda
from objects import objects
from openai import OpenAI
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

#chat lowkey cooked this one up
def clean_json_block(text: str) -> str:
    """
    Removes ```json or ``` and trailing ``` from a string containing JSON.
    Returns the clean JSON string.
    """
    # Remove ```json at the start or ``` at the start
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
    rot_error = goal_euler - robot_euler
    if np.linalg.norm(position_error) > 0.01:
        position_error = position_error / np.linalg.norm(position_error)
    if np.linalg.norm(rot_error) > 0.01:
        rot_error = rot_error / np.abs(rot_error)
    # the gains 0.001 and 0.005 match the default pos_step and rot_step in teleop
    target_position = robot_position + 0.001 * position_error
    target_euler = np.array(robot_euler + 0.005 * rot_error)
    return target_position, np.array(p.getQuaternionFromEuler(target_euler))

#setup connection to local langauge model
# Ollama runs locally and exposes an OpenAI-compatible API
client = OpenAI(
    api_key="ollama",  # required but ignored
    base_url="http://127.0.0.1:11434/v1"
)
model = "qwen2.5-coder:7b"

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
cube1 = objects.SimpleObject("cube.urdf", basePosition=[0.5, -0.3, 0.025], baseOrientation=p.getQuaternionFromEuler([0, 0, 0.7]))
cube2 = objects.SimpleObject("cube.urdf", basePosition=[0.4, -0.2, 0.025], baseOrientation=p.getQuaternionFromEuler([0, 0, -0.3]))
cube3 = objects.SimpleObject("cube.urdf", basePosition=[0.5, -0.1, 0.025], baseOrientation=p.getQuaternionFromEuler([0, 0, 0.2]))
cabinet = objects.CollabObject("cabinet.urdf", basePosition=[0.9, -0.3, 0.2], baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi]))
microwave = objects.CollabObject("microwave.urdf", basePosition=[0.5, 0.3, 0.2], baseOrientation=p.getQuaternionFromEuler([0, 0, -np.pi/2]))

# load the robot
jointStartPositions = [0.0, 0.0, 0.0, -2*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.0, 0.0, 0.04, 0.04]
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

#so the script doesnt spam us with input requests
waiting_for_input = False

#print(get_env_state())

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
        task = response.json().get("task")
        #print(f"Task: {task}")

        prompt = f'{file_content} {task}'
        response = client.chat.completions.create(
            model="qwen2.5-coder:7b",
            messages=[
                {"role": "system", "content": "You are a helpful robotics assistant."},
                {"role": "user", "content": prompt}
            ]
        )

        LLM_response = response.choices[0].message.content
        print(LLM_response)
        # parse the response to get the robot action
        # convert JSON string → Python dict
        LLM_response = clean_json_block(LLM_response)
        print(LLM_response)
        plan = json.loads(LLM_response)

        # get target position and quaternion
        robot_action = action_to_goal(robot_state["ee-position"], robot_state['ee-quaternion'],plan["target_position"], plan["target_quaternion"])
        robot_intended_action = plan["action"]
        target_object = plan["target_object"]

        print("Robot target position:", robot_action[0])
        print("Robot target quaternion:", robot_action[1])
        print("Action:", robot_intended_action)
        print("Target object:", target_object)
    else:
        #keep current robot action
        robot_action = robot_action
        #doesn't really do anything tbh
    
    alpha = 0.5 #don't move less user is too
    if DidUserAct:
        alpha = 0.2

    # blending human robot control
    target_position = (1-alpha) * human_position + alpha * np.array(robot_action[0])
    target_quaternion = (1-alpha) * human_quaternion + alpha * np.array(robot_action[1])

    #Check for table interference
    if target_position[2] + action[2] < z_buffer:
        action[2] = 0
    
    target_position = target_position + action[0:3] 

    target_quaternion = p.multiplyTransforms([0, 0, 0], p.getQuaternionFromEuler(action[3:6]),
                                                [0, 0, 0], target_quaternion)[1]
    # move to the target pose
    panda.move_to_pose(ee_position=target_position, ee_quaternion=target_quaternion)

    # open or close the gripper
    if action[6] == +1:
        panda.open_gripper()
        gripper_open = True
    elif action[6] == -1:
        panda.close_gripper()
        gripper_open = False

    
    # step the simulation
    p.stepSimulation()
    time.sleep(control_dt)