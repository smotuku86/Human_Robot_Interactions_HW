# HW2

Dylan Losey, Virginia Tech.

In this homework assignment we will use models to learn from humans.

## Install and Run

```bash

# Download
git clone https://github.com/vt-hri/HW2.git
cd HW2

# Create and source virtual environment
# If you are using Mac or Conda, modify these two lines as shown in [HW0](https://github.com/vt-hri/HW0)
python3 -m venv venv
source venv/bin/activate

# Install dependencies
# If you are using Mac or Conda, modify this line as shown in [HW0](https://github.com/vt-hri/HW0)
# Note that matplotlib is only used if you want to collect new demonstrations, and is not essential
pip install numpy pybullet matplotlib
```

## Structure

In this assignment we focus on a robot arm interacting with a cube.
Run `main.py` to visualize this environment.
Instead of reasoning about the entire state, we have broken the state down into features.
There are four features (documented in `main.py`), and you can see their values by pressing the "." key as `main.py` runs.
The human has a desired task for the robot (i.e., pick up the block, push the block in the x-axis, etc.).
The human specifies their desired task through the vector `theta`, which assigns values between [-1, +1] for each feature.
For example, if `theta=[-1.0, 0, 0, 0]` then the task is minimizing the robot's distance from the block.
Our goal will be to recover `theta` from human feedback.
To get this started, you will have access to 10 demonstrations in the `demos` folder.
You can visualize the demos by looking through images: for example, the images in `demos1` correspond to `demo1.json`.
You will develop the code in `learn_theta.py` to recover theta from human preference feedback.
Currently, `learn_theta.py` is set up to load and score demonstrations.

## Assignment

Modify the provided code to complete the following steps:
1. Write a Boltzmann human model. This model should take in a given theta and two demonstrations, and output the probability that the human selects the first demonstration as their preference.
2. Tune the Beta hyperparameter within this human model. Find a value where the human usually (but not always) selects the demonstration with a higher score.
3. Assume that the human selects demo1 as better than demo2. Use Metropolis-Hastings to estimate what theta could be based on this human preference.
4. Select a theta of your choice, and use this theta to score all 10 given demonstrations. Then sample pairs of demonstrations at random, and assume that the human always picks the one with the higher score. Extend your Metropolis-Hastings algorithm to recover theta based on the human's preference across multiple sampled pairs.
5. Take your estimated value of theta and apply it to `main.py`. Teleoperate the robot and print the score. Does the printed score increase and decrease as you would expect?