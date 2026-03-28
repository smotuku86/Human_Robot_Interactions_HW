import pybullet as p
import pybullet_data
import numpy as np
import os
import time
import torch
import pickle
from models import MLPPolicy
from robot import Panda

def get_action():
    key = p.getKeyboardEvents()
    return key

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

# load dataset to add corrections to
pickle_file_name = "HW7/Broad_dataset_1example.pkl"
data = []
with open(pickle_file_name, 'rb') as f:
    data = pickle.load(f)

# test and see how your learned policy does!
n_tests = 10
action_magnitude = 0.1
score = 0
teleop_on = False
for test_idx in range(n_tests):

    # reset the robot
    panda.reset(jointStartPositions)
    #random cube placement
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

        #Expert Overide to fix bad actions
        #this is activated based on pressing the spacebar
        key = get_action()

        if ord(" ") in key and (key[ord(" ")] & p.KEY_WAS_TRIGGERED):
            teleop_on = True
            
            action_magnitude = 0.1
            for time_idx in range (1000):
                # get the robot's position and then the cube's
                robot_state = panda.get_state()
                robot_pos = np.array(robot_state["ee-position"])
                cube_position = np.array(p.getBasePositionAndOrientation(cube)[0])
                # select the robot's action
                action = cube_position - robot_pos
                if np.linalg.norm(action) > action_magnitude:
                    action *= action_magnitude / np.linalg.norm(action)

                # store the state-action pair
                state = robot_pos.tolist() + cube_position.tolist()
                #correct behavior added to data set
                data.append(state + action.tolist())

                # move the robot with action
                panda.move_to_pose(robot_pos + action, ee_rotz=0, positionGain=0.01)
                p.stepSimulation()
                time.sleep(control_dt)

            break #skip the rest of the loop as we leave the corrected actions squence

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
        if teleop_on:
            teleop_on = False
            continue #skip score addition
        score += 1

#our score to see how well the model did
print("Score:", round(score/n_tests, 2)*100, "Percent") 

# save the updated dataset of demonstrations
pickle.dump(data, open("HW7/Broad_dataset_1example_with_corrections.pkl", "wb"))
print("dataset has this many state-action pairs:", len(data))