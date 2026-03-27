import pybullet as p
import pybullet_data
import numpy as np
import os
import time
import torch
from models import MLPPolicy
from robot import Panda


# parameters
control_dt = 1. / 240.

# create simulation and place camera
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.resetDebugVisualizerCamera(cameraDistance=1.0, 
                                cameraYaw=40.0,
                                cameraPitch=-30.0, 
                                cameraTargetPosition=[0.5, 0.0, 0.2])

narrow_dist_x = [.3, .4]
narrow_dist_y = [-.1, .1]

broad_dist_x = [.1, .6]
broad_dist_y = [-.4, .4]

# load the objects
urdfRootPath = pybullet_data.getDataPath()
plane = p.loadURDF(os.path.join(urdfRootPath, "plane.urdf"), basePosition=[0, 0, -0.625])
table = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[0.5, 0, -0.625])
cube = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), 
                  basePosition=np.random.uniform([broad_dist_x[0], broad_dist_y[0], 0.025], 
                                                 [broad_dist_x[1], broad_dist_y[1], 0.025], (3,)))


# load the robot
jointStartPositions = [0.0, 0.0, 0.0, -2*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.0, 0.0, 0.04, 0.04]
panda = Panda(basePosition=[0, 0, 0],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                jointStartPositions=jointStartPositions)

# load the trained model
model = MLPPolicy(state_dim=6, hidden_dim=64, action_dim=3)
model.load_state_dict(torch.load('HW7/Broad_model_weights_20'))
model.eval()

# test and see how your learned policy does!
n_tests = 10
action_magnitude = 0.1
score = 0
for test_idx in range(n_tests):

    # reset the robot
    panda.reset(jointStartPositions)
    #cube_position = np.random.uniform([broad_dist_x[0], broad_dist_y[0], 0.025], 
    #                                  [broad_dist_x[1], broad_dist_y[1], 0.025], (3,))
    cube_position = np.random.uniform([narrow_dist_x[0], narrow_dist_y[0], 0.025], 
                                      [narrow_dist_x[1], narrow_dist_y[1], 0.025], (3,))
    p.resetBasePositionAndOrientation(cube, cube_position, p.getQuaternionFromEuler([0, 0, 0]))

    init_cube_pos = cube_position
    # run sequence of position and gripper commands
    for time_idx in range (1000):

        # get the robot's position
        robot_state = panda.get_state()
        robot_pos = np.array(robot_state["ee-position"])

        # get the state
        state = torch.FloatTensor(robot_pos.tolist() + cube_position.tolist())

        # use the learned policy to output an action
        action = model(torch.FloatTensor(state)).detach().numpy()

        # normalize the size of the action
        if np.linalg.norm(action) > action_magnitude:
            action *= action_magnitude / np.linalg.norm(action)

        # move the robot with action
        panda.move_to_pose(robot_pos + action, ee_rotz=0, positionGain=0.01)
        p.stepSimulation()
        time.sleep(control_dt)
    
    
    #check to see if robot reached cube position
    robot_end_pos = panda.get_state()['ee-position']
    robot_2_cube = np.linalg.norm(robot_end_pos - init_cube_pos)
    #and check to see if cube moved
    cube_end_position = p.getBasePositionAndOrientation(cube)[0]
    cube_dist_moved = np.linalg.norm(cube_end_position - init_cube_pos)
    
    if robot_2_cube < 1e-2 and cube_dist_moved < 1e-2: #within 1 mm
        score += 1

print("Score:", round(score/n_tests, 2)*100, "Percent") 
