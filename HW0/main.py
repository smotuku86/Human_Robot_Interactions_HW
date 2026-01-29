import pybullet as p
import pybullet_data
import numpy as np
import os
import time
from robot import Panda


# parameters
control_dt = 1. / 240.

#Create Functions 
def move_to_pose(ee_position, angle=0.0):
    for i in range (800):
        panda.move_to_pose(ee_position=ee_position, ee_rotz=angle, positionGain=0.01)
        p.stepSimulation()
        time.sleep(control_dt)  
    return

def close_gripper():
    for i in range (300):
        panda.close_gripper()
        p.stepSimulation()
        time.sleep(control_dt)
    return

def open_gripper():
    for i in range (300):
        panda.open_gripper()
        p.stepSimulation()
        time.sleep(control_dt)
    return

def task_completed():
    print("Task Completed!")
    return

# create simulation and place camera
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.resetDebugVisualizerCamera(cameraDistance=1.0, 
                                cameraYaw=40.0,
                                cameraPitch=-30.0, 
                                cameraTargetPosition=[0.5, 0.0, 0.2])

# load the objects
urdfRootPath = pybullet_data.getDataPath()
plane = p.loadURDF(os.path.join(urdfRootPath, "plane.urdf"), basePosition=[0, 0, -0.625])
table = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[0.5, 0, -0.625])

cube1 = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), basePosition=[0.6, -0.2, 0.05])
cube2 = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), basePosition=[0.5, 0.2, 0.05])
cube3 = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), basePosition=[0.25, 0.1, 0.05])
cube4 = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), basePosition=[0.4, -0.3, 0.05])
cube5 = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), basePosition=[0.3, 0.3, 0.05])
cube6 = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), basePosition=[0.7, 0.0, 0.05])

# load the robot
jointStartPositions = [0.0, 0.0, 0.0, -2*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.0, 0.0, 0.04, 0.04]
panda = Panda(basePosition=[0, 0, 0],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                jointStartPositions=jointStartPositions)

# let the scene initialize
for i in range (300):
    p.stepSimulation()
    time.sleep(control_dt)


#LM Output
#Chat Log - https://chatgpt.com/share/697adb91-a2b0-8004-8ea5-b12ef663f3f8 

# **Execution: Build a 3-block-wide pyramid with 0.01 m spacing**
# Grasp at z = 0.05 (top of each block), lift to pre-grasp height for moves

open_gripper()
pre_grasp_height = 0.15  # safe height above table for horizontal moves

# --- Layer 1: Bottom Layer (3 blocks) ---
bottom_layer_y = 0.0
bottom_left_x = 0.25
bottom_middle_x = bottom_left_x + 0.06  # 0.31
bottom_right_x = bottom_middle_x + 0.06  # 0.37

# ---- Pick cube3 and place at bottom left ----
move_to_pose([0.25, 0.1, 0.05], angle=0)
close_gripper()
move_to_pose([0.25, 0.1, pre_grasp_height], angle=0)
move_to_pose([bottom_left_x, bottom_layer_y, pre_grasp_height], angle=0)
move_to_pose([bottom_left_x, bottom_layer_y, 0.05], angle=0)
open_gripper()
move_to_pose([bottom_left_x, bottom_layer_y, pre_grasp_height], angle=0)

# ---- Pick cube5 and place at bottom middle ----
move_to_pose([0.3, 0.3, 0.05], angle=0)
close_gripper()
move_to_pose([0.3, 0.3, pre_grasp_height], angle=0)
move_to_pose([bottom_middle_x, bottom_layer_y, pre_grasp_height], angle=0)
move_to_pose([bottom_middle_x, bottom_layer_y, 0.05], angle=0)
open_gripper()
move_to_pose([bottom_middle_x, bottom_layer_y, pre_grasp_height], angle=0)

# ---- Pick cube2 and place at bottom right ----
move_to_pose([0.5, 0.2, 0.05], angle=0)
close_gripper()
move_to_pose([0.5, 0.2, pre_grasp_height], angle=0)
move_to_pose([bottom_right_x, bottom_layer_y, pre_grasp_height], angle=0)
move_to_pose([bottom_right_x, bottom_layer_y, 0.05], angle=0)
open_gripper()
move_to_pose([bottom_right_x, bottom_layer_y, pre_grasp_height], angle=0)

# --- Layer 2: Middle Layer (2 blocks) ---
middle_layer_y = 0.0
middle_left_x = bottom_left_x + 0.03  # 0.28
middle_right_x = bottom_middle_x + 0.03  # 0.34
middle_layer_z = 0.10

# ---- Pick cube4 and place at middle left ----
move_to_pose([0.4, -0.3, 0.05], angle=0)
close_gripper()
move_to_pose([0.4, -0.3, pre_grasp_height], angle=0)
move_to_pose([middle_left_x, middle_layer_y, pre_grasp_height], angle=0)
move_to_pose([middle_left_x, middle_layer_y, middle_layer_z], angle=0)
open_gripper()
move_to_pose([middle_left_x, middle_layer_y, pre_grasp_height], angle=0)

# ---- Pick cube1 and place at middle right ----
move_to_pose([0.6, -0.2, 0.05], angle=0)
close_gripper()
move_to_pose([0.6, -0.2, pre_grasp_height], angle=0)
move_to_pose([middle_right_x, middle_layer_y, pre_grasp_height], angle=0)
move_to_pose([middle_right_x, middle_layer_y, middle_layer_z], angle=0)
open_gripper()
move_to_pose([middle_right_x, middle_layer_y, pre_grasp_height], angle=0)

# --- Layer 3: Top Layer (1 block) ---
top_layer_x = (middle_left_x + middle_right_x) / 2  # center
top_layer_y = 0.0
top_layer_z = 0.15
top_clear_height = 0.20  # raised height to avoid hitting middle layer

# ---- Pick cube6 and place at top ----
move_to_pose([0.7, 0.0, 0.05], angle=0)  # grasp cube6
close_gripper()
move_to_pose([0.7, 0.0, pre_grasp_height], angle=0)
move_to_pose([top_layer_x, top_layer_y, top_clear_height], angle=0)  # move above pyramid safely
move_to_pose([top_layer_x, top_layer_y, top_layer_z], angle=0)       # lower to place
open_gripper()
move_to_pose([top_layer_x, top_layer_y, top_clear_height], angle=0)  # lift after placement

# --- Task Complete ---
task_completed()

