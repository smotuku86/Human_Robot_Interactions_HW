import pickle
import numpy as np
import matplotlib.pyplot as plt


### states are vectors with 4 dimensions:
# x-y position of the mobile robot
# x-y position of the goal
# [x_robot, y_robot, x_goal, y_goal]

### actions are vectors with 2 dimensions:
# [delta_x_robot, delta_y_robot]

### task description
# the mobile robot is moving in an x-y plane
# we want to train this robot to move to the goal
# the goal locations are randomized, and could be different at each test run
# optimize the dataset of (state, action) pairs to teach this task

# dynamics: (state, action) -> next_state
def dynamics(state, action):
	# the goal is static
	next_state = np.copy(state)
	# move the robot by the action
	next_state[0:2] += action
	return next_state

# get an action that goes directly to the goal
def get_action(state):
	action_vec = state[2:4] - state[0:2]
	if np.linalg.norm(action_vec) > 1.0:
		action_vec /= np.linalg.norm(action_vec)
	return action_vec

# add a (state, action) pair to the dataset
def add_to_dataset(dataset, state, action):
	dataset.append(state.tolist() + action.tolist())

# visualize a (state, action) pair
def plot_state_action(state, action):
	plt.plot(state[0], state[1], 'bo')	# robot position
	plt.plot(state[2], state[3], 'ro')	# goal position
	# plot the vector for the action
	plt.plot([state[0], state[0]+action[0]], [state[1], state[1]+action[1]], 'b-')
	plt.axis("equal")
	plt.savefig("state_action.png")

### first pass solution (you will want to improve on this...)
def get_dataset():
	dataset = []
	for _ in range(100):
		# pick a state uniformly at random
		state = np.random.uniform(-10, 10, 4)
		# calculate action towards closest goal
		action = get_action(state)
		# add the state-action pair to the dataset
		add_to_dataset(dataset, state, action)
		# # plot the state-action pair
		# plot_state_action(state, action)

	pickle.dump(dataset, open("dataset.pkl", "wb"))
	print("dataset has this many state-action pairs:", len(dataset))

if __name__ == "__main__":
	get_dataset()