# HW8

Dylan Losey, Virginia Tech.

In this homework assignment we will use behavior cloning to learn the robot's policy using image observations.

## Install and Run

```bash

# Download
git clone https://github.com/vt-hri/HW8.git
cd HW8

# Create and source virtual environment
# If you are using Mac or Conda, modify these two lines as shown in [HW0](https://github.com/vt-hri/HW0)
python3 -m venv venv
source venv/bin/activate

# Install dependencies
# If you have torch installed for HW7, you can skip this code
# If you are using Mac or Conda, modify this line as shown in [HW7](https://github.com/vt-hri/HW0)
pip install numpy pybullet torch
```

## Assignment

Your goal is to familiarize yourself with the process of learning a robot policy, and to practice this concept by implementing behavior cloning. 
The robot should learn to reach towards blocks that are randomly placed on the table.
You will complete the following steps:
1. Familiarize yourself with the code. Determine how many demonstrations you need to learn a robot policy from image observations.
2. Train your robot to reach for red blocks that are randomly initialized on the table. Then test your policy on a different distribution (e.g., block color is blue).
4. Modify the get_dataset.py code so that the block color is different in each demonstration. Train your robot to reach for the blocks. Then repeat the previous step by testing the policy to reach for blue blocks. Record your observations about how the data distribution affects the robot's learned behaviors.

From Assignment
1. a text document that explains how the performance of the learned policy changes with the number of demos. Does it take more or less data than in HW7? Explain why that is the case.
2. videos of your robot executing the policy it learned on a narrow training distribution (single color cubes) and a wide training distribution (multiple cube colors). Provide documentation so that we can tell which video is which.
3. a text document that explains how the training distribution affects the performance of the policy in a wide test distribution.
the relevant code files to reproduce your results