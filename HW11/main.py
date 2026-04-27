import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize


# vehicle dynamics
def dynamics(state, action):
    delta = 1.0 * np.array([np.cos(action), np.sin(action)])
    new_state = state + delta
    return new_state

# trajectory rollout
def rollout(initial_state, actions):
    state = np.copy(initial_state)
    xi = [state.tolist()]
    for a in actions:
        state = dynamics(state, a)
        xi.append(state.tolist())
    return np.array(xi)

# human cost function
def human_cost(human_actions, initial_human_state, initial_robot_state, robot_actions):
    human_trajectory = rollout(initial_human_state, human_actions)
    robot_trajectory = rollout(initial_robot_state, robot_actions)
    cost = 0.
    for idx, x in enumerate(zip(human_trajectory, robot_trajectory)):
        human_state = x[0]
        robot_state = x[1]
        cost += 5.0/np.linalg.norm(human_state - robot_state)
        cost += -human_state[0]
    return cost

# optimize the human's actions
def optimize_human(initial_human_state, initial_robot_state, human_actions, robot_actions):
    result = minimize(
            fun=human_cost,
            x0=human_actions, 
            args=(initial_human_state, initial_robot_state, robot_actions),
            method='L-BFGS-B',
            bounds=[(-np.pi, np.pi) for _ in range(len(human_actions))]
        )
    return result.x

# robot cost function
def robot_cost(robot_actions, initial_human_state, initial_robot_state, human_actions):

    human_trajectory = rollout(initial_human_state, human_actions)
    robot_trajectory = rollout(initial_robot_state, robot_actions)
    cost = 0.
    for idx, x in enumerate(zip(human_trajectory, robot_trajectory)):
        human_state = x[0]
        robot_state = x[1]
        cost += -human_state[0]
        cost += 1.0 / np.linalg.norm(human_state - robot_state)
        cost += robot_state[0]
    return cost

# optimize the robot's actions
def optimize_robot(initial_human_state, initial_robot_state, human_actions):
    result = minimize(
        fun=robot_cost,
        x0=np.zeros_like(human_actions),
        args=(initial_human_state, initial_robot_state, human_actions),
        method='L-BFGS-B',
        bounds=[(-np.pi, np.pi)] * len(human_actions)
    )
    return result.x

# stackelberg robot cost function (robot is leader, human is follower
def stackelberg_robot_cost(robot_actions, initial_human_state, initial_robot_state):
    human_actions = optimize_human(
        initial_human_state, initial_robot_state,
        np.zeros(len(robot_actions)),
        robot_actions
    )
    return robot_cost(robot_actions, initial_human_state, initial_robot_state, human_actions)

# stackelberg human cost function (human is leader, robot is follower)
def stackelberg_human_cost(human_actions, initial_human_state, initial_robot_state):
    robot_actions = optimize_robot(
        initial_human_state, initial_robot_state,
        human_actions
    )
    return human_cost(human_actions, initial_human_state, initial_robot_state, robot_actions)

# plot the trajectories
def plot_trajectory(filename, initial_human_state, initial_robot_state, human_actions, robot_actions):
    plt.figure()
    human_trajectory = rollout(initial_human_state, human_actions)
    robot_trajectory = rollout(initial_robot_state, robot_actions)
    plt.plot(human_trajectory[:,0], human_trajectory[:,1], 'bo-', label='Human')
    plt.plot(robot_trajectory[:,0], robot_trajectory[:,1], 'ro-', label='Robot')

    # Number each point along the human trajectory
    for i, (x, y) in enumerate(human_trajectory):
        plt.annotate(str(i), (x, y), textcoords="offset points", xytext=(5, 5), color='blue', fontsize=8)

    # Number each point along the robot trajectory
    for i, (x, y) in enumerate(robot_trajectory):
        plt.annotate(str(i), (x, y), textcoords="offset points", xytext=(5, -10), color='red', fontsize=8)

    plt.legend()
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.title('Trajectory Comparison')
    plt.savefig(filename)
    plt.close()

#                                       Robot leader Plot
# initialize the vehicles
initial_human_state = np.array([-7.0, 0.])
initial_robot_state = np.array([-6.0, 0.1])
human_actions = np.zeros(5)

robot_result = minimize(
    fun=stackelberg_robot_cost,
    x0=np.zeros(5),
    args=(initial_human_state, initial_robot_state),
    method='L-BFGS-B',
    bounds=[(-np.pi, np.pi)] * 5
)

robot_actions = robot_result.x
human_actions = optimize_human(initial_human_state, initial_robot_state, human_actions, robot_actions)
filename = 'HW11/stackelberg_robot_leader_helping.png'
plot_trajectory(filename, initial_human_state, initial_robot_state, human_actions, robot_actions)


#                                       Human leader Plot

# initialize the vehicles
initial_human_state = np.array([-7.0, 0.])
initial_robot_state = np.array([-6.0, 0.1])
human_actions = np.zeros(5)

human_result = minimize(
    fun=stackelberg_human_cost,
    x0=np.zeros(5),
    args=(initial_human_state, initial_robot_state),
    method='L-BFGS-B',
    bounds=[(-np.pi, np.pi)] * 5
)

human_actions = human_result.x
robot_actions = optimize_robot(initial_human_state, initial_robot_state, human_actions)
filename = 'HW11/stackelberg_human_leader_helping.png'
plot_trajectory(filename, initial_human_state, initial_robot_state, human_actions, robot_actions)