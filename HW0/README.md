# HRI: Homework 0

Dylan Losey, Virginia Tech.

In this homework assignment we will simulate a robot arm.

## Install and Run

### Ubuntu

```bash

# Download
git clone https://github.com/vt-hri/HW0.git
cd HW0

# Create and source virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install numpy pybullet

# Run the script
python main.py
```

### Windows

One way to run this is with WSL, [Windows Subsystem for Linux](https://learn.microsoft.com/en-us/windows/wsl/install).

1. Install WSL. Windows provides instructions [here](https://learn.microsoft.com/en-us/windows/wsl/setup/environment). I also recommend this [tutorial](https://www.youtube.com/watch?v=-Wg2r1lWrTc). Once installed, make sure to update and upgrade packages using:
```bash

sudo apt update && sudo apt upgrade
sudo apt install python3-pip python3-venv

```

2. Open an Ubuntu terminal. Right click, and paste the code shown above in the [Ubuntu](#ubuntu) section. After the packages are installed, you should see the [Expected Output](#expected-output).

3. Install a text editing software. I use [Sublime](https://www.sublimetext.com/download), but there are many good options. You will use this text editing software to write your code.

### Mac

You can use [Anaconda](https://www.anaconda.com/). Follow the instructions [here](https://www.anaconda.com/download/success). Once installed, make sure conda is setup correctly by running 
```bash
conda
```

You might have to source the .bash (or .zsh) file depending on which bash script the terminal uses. Or, simply close and re-open the terminal.

```bash
# Create a new conda env with python version 3.10.
conda create -n venv python=3.10

# Install pybullet from conda
conda install pybullet

# You can use pip to install all the other packages.
pip install numpy

# Run the script
python main.py
```

## Expected Output

<img src="env.gif" width="750">
