import numpy as np
import matplotlib.pyplot as plt


# plot the human and robot (saves as png file)
def plot_circle(theta_human, theta_robot, step):
    position_human = theta_to_position(theta_human)
    position_robot = theta_to_position(theta_robot)
    circle_angles = np.linspace(0, 2*np.pi, 30)
    plt.plot(np.cos(circle_angles), np.sin(circle_angles), 'k-')
    plt.plot(position_human[0], position_human[1], 'bo', markersize=15)
    plt.plot(position_robot[0], position_robot[1], 'rs', markersize=10)
    plt.axis("equal")
    # timestep labels
    plt.text(position_human[0] + 0.00,
             position_human[1] + 0.15,
             f"H{step}")

    plt.text(position_robot[0] + 0.1,
             position_robot[1] - 0.15,
             f"R{step}")
    

# convert angle (in radians) to position on circle
def theta_to_position(theta):
    return np.array([np.cos(theta), np.sin(theta)])

# error between the human and robot positions on circle
def cost_function(theta_human, theta_robot):
    position_human = theta_to_position(theta_human)
    position_robot = theta_to_position(theta_robot)
    return np.linalg.norm(position_human - position_robot)

# rollout a sequence of human and robot actions
def rollout(human_actions, robot_actions, plot=False, savename="HW13/env.png"):
    theta_human = 0.
    theta_robot = 0.
    cost = 0
    for idx, x in enumerate(zip(human_actions, robot_actions)):
        theta_human += x[0]
        theta_robot += x[1]
        cost += cost_function(theta_human, theta_robot)
        if plot:
            plot_circle(theta_human, theta_robot, step=idx)
    if plot:
        plt.legend(["Circle", "Human", "Robot"])        
        plt.savefig(savename)
        plt.cla()
    return cost

# compute the gradient for the human
def gradient_human(human_actions, robot_actions, delta=0.01):
    cost = rollout(human_actions, robot_actions)
    grad = np.zeros(human_actions.shape)
    for idx in range(len(grad)):
        human_actions1 = np.copy(human_actions)
        human_actions1[idx] += delta
        cost1 = rollout(human_actions1, robot_actions)
        grad[idx] = (cost1 - cost) / delta
    return grad

# compute the gradient for the robot
def gradient_robot(human_actions, robot_actions, delta=0.01):
    cost = rollout(human_actions, robot_actions)
    grad = np.zeros(robot_actions.shape)
    for idx in range(len(grad)):
        robot_actions1 = np.copy(robot_actions)
        robot_actions1[idx] += delta
        cost1 = rollout(human_actions, robot_actions1)
        grad[idx] = (cost1 - cost) / delta
    return grad


### main code: your changes below!

# initial action plans for both agents
human_actions = np.array([0., 0., 0., 0., 0.])
robot_actions = np.array([0., 0., 0., 0., 0.])

# example of the human updating their behavior in response to the robot
human_alpha = .1
robot_alpha = .05
human_actions1 = np.copy(human_actions)
robot_actions1 = np.copy(robot_actions)

for timestep in range(10):
    if timestep % 2 == 1:
        robot_grad = gradient_robot(human_actions1, robot_actions1)
        robot_actions1 -= robot_alpha * robot_grad
        robot_actions1 = np.clip(robot_actions1, -0.5, 0.5)
    else:
        human_grad = gradient_human(human_actions1, robot_actions1)
        human_actions1 += human_alpha * human_grad
        human_actions1 = np.clip(human_actions1, -0.5, 0.5)

# print the resulting actions and then plot them
print(human_actions1)
print(robot_actions1)
rollout(human_actions1, robot_actions1, plot=True, savename="HW13/env_H.1_R.05.png")
