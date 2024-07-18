
# Rotary Inverted Pendulum - PID Control

## Overview

This project demonstrates the implementation of a PID (Proportional-Integral-Derivative) controller to stabilize a rotary inverted pendulum using the MuJoCo physics engine. The PID controller is a simple yet effective control strategy widely used in industrial control systems.

---

## 📁 Project Structure

- `Rotary_Inverted_Pendulum_PID.py`: Python script implementing the PID controller using MuJoCo for simulation.

---

## 🚀 Getting Started

### Prerequisites

Ensure you have the following installed:

- Python 3.x
- MuJoCo
- mujoco-py
- GLFW
- NumPy
- Matplotlib

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/alirezarezaeei78/rotary-inverted-pendulum.git
   cd Rotary_Inverted_Pendulum_PID
   ```

2. Install the required Python packages:
   ```bash
   pip install -r requirements.txt
   ```

### Running the Simulation

Run the PID control script:
```bash
python Rotary_Inverted_Pendulum_PID.py
```

This will start the simulation, and you will see the rotary inverted pendulum being controlled by the PID controller.

---

## 🛠️ Code Explanation

### PID Controller Parameters

The PID controller is defined with the following parameters:

```python
Kp = 100.0
Ki = 1.0
Kd = 20.0
```

These parameters can be tuned to achieve the desired performance.

### PID Control Function

The `pid_control` function calculates the control signal based on the target and current positions of the pendulum:

```python
def pid_control(target, current, dt):
    global integral, previous_error
    error = target - current
    integral += error * dt
    derivative = (error - previous_error) / dt
    previous_error = error
    return Kp * error + Ki * integral + Kd * derivative
```

### Main Control Loop

The main control loop initializes the MuJoCo environment and runs the simulation:

```python
while not glfw.window_should_close(window):
    time_prev = data.time
    while data.time - time_prev < 1.0 / 60.0:
        mj.mj_step(model, data)
    if data.time >= simend:
        break
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    glfw.swap_buffers(window)
    glfw.poll_events()
glfw.terminate()
```

---

## 📊 Results

During the simulation, the terminal will output the positions of the pendulum joints, which helps in understanding the performance of the PID controller:

```plaintext
qpos0:0.13778828708005658 qpos1:0.015267968412768972
qpos0:0.13783148493428932 qpos1:0.012980073131278034
...
```

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 📞 Contact

If you have any questions or need further assistance, feel free to contact:

- Email: alireza78@email.kntu.ac.ir
- GitHub: [alirezarezaeei78](https://github.com/alirezarezaeei78)
