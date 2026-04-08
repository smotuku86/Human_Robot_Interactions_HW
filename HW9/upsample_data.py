import numpy as np
import pickle

# load the dataset we want to upsample
loadname = "HW9/datasets/default_dataset.pkl"
data = pickle.load(open(loadname, "rb"))
data = np.array(data)

# initialize a new dataset
dataset = []
# for each element in the dataset
for x in data:
	# get the state elements
	robot_pos = x[0:3]
	cabinet_pos = x[3:6]
	action = x[6:9]
	# find the next state
	next_robot_pos = robot_pos + action
	# sample nearby robot positions, and find the corresponding actions
	# which are to get back to a known state in hte training set
	for _ in range(10):
		robot_pos1 = robot_pos + np.random.normal(0, 0.01, 3)
		action1 = next_robot_pos - robot_pos1
		dataset.append(robot_pos1.tolist() + cabinet_pos.tolist() + action1.tolist())

# save the upsampled dataset
pickle.dump(dataset, open("HW9/datasets/default_dataset_upsampled.pkl", "wb"))
print("original dataset has this many state-action pairs:", len(data))
print("new dataset has this many state-action pairs:", len(dataset))