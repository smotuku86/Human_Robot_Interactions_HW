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
def move_robots(robot1_state, robot2_state, action1, action2, step_count, num_steps):
    # move to chosen blocks
    score = [0, 0]
    num_steps = num_steps - 1  #bc the forloop is range(n) - it only counts to n-1
    
    for idx in range(500):
        if action1 is -1:
            panda1.move_to_joint(positionGain=0.01)
        else:
            robot1_position = np.array(robot1_state["ee-position"])
            robot1_rotz = robot1_state["ee-euler"][2]
            cube1 = get_cube_state(cubes, action1)
            cube1_positon = np.array(cube1["position"])
            cube1_rotz = cube1["euler"][2]
            position2move2 = robot1_position + step_count/num_steps * ( cube1_positon - robot1_position)
            rotz2move2 = robot1_rotz + step_count/num_steps * (cube1_rotz - robot1_rotz)

            panda1.move_to_pose(ee_position= position2move2, ee_rotz=rotz2move2, positionGain=0.01)

        if action2 is -1:
            panda2.move_to_joint(positionGain=0.01)
        else:
            robot2_position = np.array(robot2_state["ee-position"])
            robot2_rotz = robot2_state["ee-euler"][2]
            cube2 = get_cube_state(cubes, action2)
            cube2_positon = np.array(cube2["position"])
            cube2_rotz = cube2["euler"][2]
            position2move2 = robot2_position + step_count/num_steps * (cube2_positon - robot2_position)
            rotz2move2 = robot2_rotz + step_count/num_steps * (cube2_rotz - robot2_rotz)

            panda2.move_to_pose(ee_position= position2move2, ee_rotz=rotz2move2, positionGain=0.01)
        
        p.stepSimulation()
        time.sleep(control_dt) 
    # grasp blocks
    if (step_count == num_steps):
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
    
# function that returns the goal positions and rotation around z for each object
def get_object_goals(cubes):
    goals = {}
    for cube in cubes:
        goals[cube]={}
        cube_state = get_cube_state(cubes, cube)
        goals[cube]["position"] = cube_state["position"]
        goals[cube]["rotz"] = cube_state["euler"][2]
    return goals

# Find the different between gripper orientation and possible goal orientation
def angle_diff(a, b):
    # returns smallest signed angle difference in [-pi, pi]
    return np.arctan2(np.sin(a - b), np.cos(a - b))

def predict_goal(initial_robot_state, current_robot_state, goals, beta = 1.0, rot_scale = 0.0):
    #predict what object the robot is trying to get 
    #Returns a dict with the chance of each object being the goal
    #initial_robot_state and current_robot_state are both lists of length 4, with the first 3 values being the position and the last being rotation around z (euler)

    goal_predictions = {}

    #get and seperate the position and rotation values for the initial and current robot state
    initial_robot_state_position = np.array(initial_robot_state["ee-position"])
    initial_robot_state_rotz = initial_robot_state["ee-euler"][2]
    current_robot_state_position = np.array(current_robot_state["ee-position"])
    current_robot_state_rotz = current_robot_state["ee-euler"][2]

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
physicsClient = p.connect(p.GUI) #no gui, run faster do p.DIRECT
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
#print(cubes)

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

    print(get_object_goals(cubes))

    initial_robot1_state = panda1.get_state()
    initial_robot2_state = panda2.get_state()

    print(initial_robot1_state)
    #we control robot 1
    #robot 2 choose a random cube and goes for it 

    # choose which cube each robot should pick up
    # robot 2 is random
    r2_chosen_cube = np.random.randint(0, n_cubes)


    #design a strat that does not collide with robot2 
    # (not get the same cube or anything near it)

    #get goals info on robot 2
    goals = get_object_goals(cubes)

    #I want robot 1 to decide "real time" when to change its cube
    # going to split pathing into 10 steps to make this happen in a simple way
    #
    # move robots will be modified to take steps to goal rather than travel all at once

    steps = 10
    iteration_score = [0,0]
    for step in range(steps):
        print("Step count:", step)

        #try to predict the goal robot 2 is going for
        current_robot2_state = panda2.get_state()
        robot2_predictions = predict_goal(current_robot2_state, initial_robot2_state, goals, beta = 10.0, rot_scale = 0.0)
        robot2_predictions_sorted = sorted(robot2_predictions, key=robot2_predictions.get)
                                    #sorted most liekly to least
        #make robot 1 pick a cube that robot 2 is unlikely to get
        print("predictions:", robot2_predictions_sorted)
        robot1_chosen_cube = robot2_predictions_sorted[-1]

        action1 = robot1_chosen_cube
        action2 = r2_chosen_cube

        print("r1:", action1)
        print("r2:", action2)

        # robots try to grasp the chosen cubes and lift them
        iteration_score = move_robots(initial_robot1_state, initial_robot2_state, action1, action2, step, steps)
                                    #if wonky try current states


    '''
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
    '''
    total_score[0] += iteration_score[0]
    total_score[1] += iteration_score[1]
    print("here is the score from that iteration: ", iteration_score)
    print("after", iteration+1, "rounds, the score is: ", total_score)

    # reset cubes to random positions for next round
    for cube in cubes:
        cube_init_position = np.random.uniform([-0.15, -0.3, 0.025], [0.15, 0.3, 0.025], (3,))
        cube_init_angle = np.random.uniform([0, 0, 0], [0, 0, np.pi/2], (3,))
        p.resetBasePositionAndOrientation(cubes[cube], cube_init_position, p.getQuaternionFromEuler(cube_init_angle))