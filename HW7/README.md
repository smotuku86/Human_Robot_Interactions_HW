# HW7

Dylan Losey, Virginia Tech.

In this homework assignment we will use behavior cloning to learn the robot's policy.

## Install and Run

```bash

# Download
git clone https://github.com/vt-hri/HW7.git
cd HW7

# Create and source virtual environment
# If you are using Mac or Conda, modify these two lines as shown in [HW0](https://github.com/vt-hri/HW0)
python3 -m venv venv
source venv/bin/activate

# Install dependencies
# If you are using Mac or Conda, modify this line as shown in [HW0](https://github.com/vt-hri/HW0)
pip install numpy pybullet torch
```

## Assignment

Your goal is to familiarize yourself with the process of learning a robot policy, and to practice this concept by implementing behavior cloning. 
The robot should learn to reach towards blocks that are randomly placed on the table.
Modify your code as needed to complete the following steps:
1. Familiarize yourself with the code. Determine how to modify each of the following parameters, and explain what changing those parameters will do:
	- The number of hidden layers and the size of each layer
	- The number of epochs and the learning rate
	- The batch size
2. Train your robot to pick up blocks that are initialized in a small distribution (e.g., in a radius of 0.1 meters). Then test your policy on a larger distribution (e.g., blocks anywhere on the table).
3. Repeat the previous step, but now switch the roles. Train on a wide distribution, and test on a narrow distribution. Consider your results for both of these steps. What does this mean about the training distribution?
4. Vary the number of examples you use to train the robot's policy. Is more examples always better? How does the robot work with just one or two examples in the training set?
5. Implement a version of DAgger where users can interrupt the robot's learned policy and correct its behavior. This corrected behavior should be added to the dataset and used for retraining the policy. Instead of having an actual "user" correct the robot, you can have an expert policy (like the one in get_dataset) take over to provide the correction.