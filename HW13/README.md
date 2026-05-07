# HW13

Dylan Losey, Virginia Tech.

In this homework assignment we will explore how competitive agents can co-adapt.

## Install and Run

```bash

# Download
git clone https://github.com/vt-hri/HW13.git
cd HW13

# Create and source virtual environment
# If you are using Mac or Conda, modify these two lines as shown in [HW0](https://github.com/vt-hri/HW0)
# If you have previously created a virtual environment with torch, you can just source that environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
# If you are using Mac or Conda, modify this line as shown in [HW0](https://github.com/vt-hri/HW0)
pip install numpy matplotlib

# Run the script
python main.py
```
## Expected Output

<img src="env.png" width="750">

## Assignment

You are given the code for two agents (say a human and a robot) playing hide and seek.
The agents are constrained to move around a circle.
The human (in blue) is hiding, and the robot (in red) is seeking.
Formally, the human tries to maximize their distance from the robot, and the robot seeks to minimize their distance from the robot.
Both agents will co-adapt as they come up with better ways to play this game.
Complete the following steps:

1. Describe how an agent modifies their actions. What changes in the code will cause the agent to make "larger" or "smaller" changes?
2. What happens if the two agents take turns improving their strategies? Does the behavior converge? What happens if they have different "rates" for updating their actions? Modify the code to explore these questions.
3. What happens if they both adapt at the same time? As before, consider whether the team converges to a joint strategy, and how the interaction changes if they update at different rates. Modify the code to implement this simultaneous co-adaptation.
4. Imagine that the robot knows how the human will adapt. Progam the robot so that it is aware of the human's learning rule. How can the robot leverage this information?

1. a text document that explains how learning rates affect the ways that agents co-adapt.
2. a text document that explains how agents can leverage their knowledge of adaptation to stay "one step ahead" of their opponents.
3. the main.py file for your final version
4. png images of co-adaptation when the robot is aware of the human's learning rule