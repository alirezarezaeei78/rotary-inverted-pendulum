
# Rotary Inverted Pendulum LQR Controller

## Overview

This section of the project focuses on the implementation of the Linear Quadratic Regulator (LQR) controller for stabilizing a rotary inverted pendulum using MuJoCo, a physics engine for detailed and efficient rigid body simulations.

---

## 📂 Directory Structure

- `Rotary_Inverted_Pendulum_LQR.py` - Python script implementing the LQR controller.

---

## 📜 Rotary_Inverted_Pendulum_LQR.py

### Description

The `Rotary_Inverted_Pendulum_LQR.py` script is designed to control a rotary inverted pendulum using an LQR controller. The script includes functions to define the system dynamics, linearize the system, and compute the control law.

### Key Components

1. **Imports and Initialization**

   The script begins with importing necessary libraries and initializing variables.

   ```python
   import mujoco as mj
   from mujoco.glfw import glfw
   import matplotlib.pyplot as plt
   import numpy as np
   import os
   import control
   import math
   ```

2. **System Dynamics Function**

   Defines the system dynamics using the state and control input.

   ```python
   def f(x, u):
       ...
   ```

3. **Linearization Function**

   Linearizes the system around the operating point.

   ```python
   def linearize():
       ...
   ```

4. **LQR Controller Initialization**

   Initializes the LQR controller by computing the gain matrix \( K \).

   ```python
   def init_controller(model, data):
       ...
   ```

5. **Control Function**

   Applies the control input to the system based on the state feedback.

   ```python
   def controller(model, data):
       ...
   ```

6. **GLFW Callbacks and Main Loop**

   Sets up GLFW callbacks for user interaction and runs the main simulation loop.

   ```python
   def keyboard(window, key, scancode, act, mods):
       ...
   
   def mouse_button(window, button, act, mods):
       ...
   
   def mouse_move(window, xpos, ypos):
       ...
   
   def scroll(window, xoffset, yoffset):
       ...
   
   # Initialize GLFW and create window
   glfw.init()
   window = glfw.create_window(1200, 900, "Demo", None, None)
   ...
   
   # Main simulation loop
   while not glfw.window_should_close(window):
       ...
   glfw.terminate()
   ```

### Usage

1. **Setup MuJoCo Environment**

   Ensure that MuJoCo is installed and the `rotary inverted pendulum.xml` model file is in the same directory as the script.

2. **Install Required Packages**

   Install the required Python packages using the provided `requirements.txt`.

   ```bash
   pip install -r requirements.txt
   ```

3. **Run the Script**

   Execute the script to run the LQR controller simulation.

   ```bash
   python Rotary_Inverted_Pendulum_LQR.py
   ```

### Example Output

The script outputs the state of the pendulum during the simulation, including position and velocity, as well as the applied control inputs.

```plaintext
qpos0:0.13778828708005658 qpos1:0.015267968412768972
qpos0:0.13783148493428932 qpos1:0.012980073131278034
...
```

---

## Dependencies

- MuJoCo
- GLFW
- NumPy
- Matplotlib
- Python-Control


---

## Contact

For any questions or issues, please contact:

Email: alireza78@email.kntu.ac.ir
