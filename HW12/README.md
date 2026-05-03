# HW12

Dylan Losey, Virginia Tech.

In this homework assignment we will explore how to design training sets for robot learners.

## Install and Run

```bash

# Download
git clone https://github.com/vt-hri/HW12.git
cd HW12

# Create and source virtual environment
# If you are using Mac or Conda, modify these two lines as shown in [HW0](https://github.com/vt-hri/HW0)
# If you have previously created a virtual environment with torch, you can just source that environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
# If you are using Mac or Conda, modify this line as shown in [HW0](https://github.com/vt-hri/HW0)
pip install numpy torch matplotlib

# Run the script
python main.py
```

## Assignment

You are given the code for a mobile robot that is learning to reach a goal.
The robot is a point moving in an x-y plane, and the goal is static.
We will leave the learning algorithm fixed: our goal is to design a minimal dataset that teaches the robot the task.
Complete the following steps:

1. Go through the pipeline of collecting data, training, and testing. Make sure you understand how the code functions.
2. Focus on `main.py`. Think about what sorts of data would be "best" for teaching the task. Discuss those strategies with your peers.
3. Design your own dataset. Can you reach <1 unit of average error with 20 or less demonstrations?
4. Try to extract general principles for the data needed to teach the robot. What types of states are more informative than others?

1. a text document that explains how the code functions.
2. a text document that explains the strategies you tried for collecting data, and why you thought those strategies would be effective.
3. the main.py file for your final version, the final dataset.pkl you obtained, and a list of the number of datapoints & the average test error.
4. the image state_action.png for your final dataset; this image should "show" what your dataset contains.
5. a text document that explains what dataset you found to be most effective.
6. a text document that connects your results to broader properties for robot teaching: what types of data do we want from humans, and what can robots do to try and obtain that more efficient data?