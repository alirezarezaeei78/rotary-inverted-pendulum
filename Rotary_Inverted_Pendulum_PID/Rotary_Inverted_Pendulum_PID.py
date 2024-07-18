import mujoco as mj
from mujoco.glfw import glfw
import matplotlib.pyplot as plt
import numpy as np
import os

xml_path = 'rotary inverted pendulum.xml' # XML file path
simend = 15 # Simulation time

# PID controller parameters
Kp = 100.0
Ki = 1.0
Kd = 20.0
# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# Initialize PID controller variables
integral = np.zeros(2)
previous_error = np.zeros(2)

def pid_control(target, current, dt):
    global integral, previous_error
    error = target - current
    integral += error * dt
    derivative = (error - previous_error) / dt
    previous_error = error
    return Kp * error + Ki * integral + Kd * derivative

def controller(model, data):
    global integral, previous_error

    print(f"qpos0:{data.qpos[0]} qpos1:{data.qpos[1]}")

    target = np.array([0.0, 0.0])  # Target positions for the joints
    current = np.array([data.qpos[0], data.qpos[1]])
    dt = model.opt.timestep
    control_signal = pid_control(target, current, dt)
    data.ctrl[0] = control_signal[0]
    data.ctrl[1] = control_signal[1]
      #2 apply disturbance torque
    tau_disturb_mean = 0
    tau_disturb_dev = 150
    tau_d0 = np.random.normal(tau_disturb_mean,tau_disturb_dev)
    tau_d1 = np.random.normal(tau_disturb_mean,0.25*tau_disturb_dev)
    data.qfrc_applied[0] = tau_d0
    data.qfrc_applied[1] = tau_d1

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    global button_left, button_middle, button_right
    button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    global lastx, lasty, button_left, button_middle, button_right
    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos
    if not button_left and not button_middle and not button_right:
        return
    width, height = glfw.get_window_size(window)
    PRESS_LEFT_SHIFT = glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT
    if button_right:
        action = mj.mjtMouse.mjMOUSE_MOVE_H if mod_shift else mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        action = mj.mjtMouse.mjMOUSE_ROTATE_H if mod_shift else mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, dx / height, dy / height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 * yoffset, scene, cam)

dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname, xml_path)
xml_path = abspath

model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)
cam = mj.MjvCamera()
opt = mj.MjvOption()

glfw.init()
window = glfw.create_window(1200, 900, "PID Control Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

cam.azimuth = 35
cam.elevation = -15
cam.distance = 7
cam.lookat = np.array([2.0, 1.5, 1.0])

mj.set_mjcb_control(controller)

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