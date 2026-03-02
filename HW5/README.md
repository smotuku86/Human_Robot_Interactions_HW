# HW5

Dylan Losey, Virginia Tech.

In this homework assignment we will develop an assistive robot arm.

## Install and Run

```bash

# Download
git clone https://github.com/vt-hri/HW5.git
cd HW5

# Create and source virtual environment
# If you are using Mac or Conda, modify these two lines as shown in [HW0](https://github.com/vt-hri/HW0)
python3 -m venv venv
source venv/bin/activate

# Install dependencies
# If you are using Mac or Conda, modify this line as shown in [HW0](https://github.com/vt-hri/HW0)
pip install numpy pybullet

# Run the script
python main.py
```

## Expected Output

<img src="env.png" width="750">

## (OPTIONAL) GPT-OSS in Python

Virginia Tech hosts a language model and every student has access to it. 
However, API access is restricted to the VT Campus VPN.
To set up your free access, follow the instructions [here](https://docs.arc.vt.edu/ai/011_llm_api_arc_vt_edu.html) and get your API key.
Remember to keep your key private.

## (OPTIONAL) Connecting to GPT-OSS

Paste your API key into line 5 of `lm-test.py`.
Then install the OpenAI library and run your code:
```bash
# Install dependency
pip install openai

# Run the lm-test script
python3 lm-test.py
```
Try editing the system prompt and user prompt in `lm-test.py`.
Once you have this working, you can bring it into `main.py` to help create your assistive robot.

## Assignment

Your goal is to create a system where everyday users can make the robot perform assistive tasks of their choice. 
For example, perhaps one user wants the robot to "put the cube in the microwave." 
The user task are open ended, and a good assistive robot should be able to help for a wide variety of tasks.
Modify the code as needed to complete the following steps:
1. Brainstorm ways for everyday users to make the robot do what they want. List the desired characteristics for your assistive robot.
2. Implement your assistive strategy.
3. Define performance metrics for your system. Have a friend test your system, and compare their performance with and without your assistance strategy.
4. Use statistical analysis to determine whether your strategy makes a significant improvement (e.g., a t-test or ANOVA).