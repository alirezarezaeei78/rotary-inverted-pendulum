import mujoco as mj
from mujoco.glfw import glfw
import matplotlib.pyplot as plt
import numpy as np
import os
from collections import deque

# PyTorch
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Normal

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print(f"Device: {device}")

xml_path = 'rotary inverted pendulum.xml'  
simend = 15  # Simulation time
print_camera_config = 0  # Set to 1 to print camera config

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

class Policy(nn.Module):
    def __init__(self, s_size, a_size, h_size):
        super(Policy, self).__init__()
        self.fc1 = nn.Linear(s_size, h_size)
        self.fc2 = nn.Linear(h_size, h_size)
        self.fc_mean = nn.Linear(h_size, a_size)
        self.fc_log_std = nn.Linear(h_size, a_size)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        mean = self.fc_mean(x)
        log_std = self.fc_log_std(x)
        std = torch.exp(log_std)
        return mean, std

    def act(self, state):
        state = torch.from_numpy(state).float().unsqueeze(0).to(device)
        mean, std = self.forward(state)
        dist = Normal(mean, std)
        action = dist.sample()
        action_log_prob = dist.log_prob(action).sum()
        return action.cpu().detach().numpy()[0], action_log_prob

def reinforce(policy, optimizer, n_training_episodes, max_t, gamma, print_every):
    scores_deque = deque(maxlen=100)
    scores = []
    avg_scores = []  # For plotting
    for i_episode in range(1, n_training_episodes + 1):
        saved_log_probs = []
        rewards = []
        mj.mj_resetData(env.model, env.data)
        state = np.concatenate([env.data.qpos, env.data.qvel])
        for t in range(max_t):
            action, log_prob = policy.act(state)
            saved_log_probs.append(log_prob)
            env.data.ctrl[:] = action
            mj.mj_step(env.model, env.data)
            state = np.concatenate([env.data.qpos, env.data.qvel])
            reward = compute_reward(state) 
            rewards.append(reward)
            if env.data.time >= simend:
                break
        scores_deque.append(sum(rewards))
        scores.append(sum(rewards))
        returns = deque(maxlen=max_t)
        n_steps = len(rewards)
        for t in range(n_steps)[::-1]:
            disc_return_t = (returns[0] if len(returns) > 0 else 0)
            returns.appendleft(gamma * disc_return_t + rewards[t])
        eps = np.finfo(np.float32).eps.item()
        returns = torch.tensor(returns).to(device)
        returns = (returns - returns.mean()) / (returns.std() + eps)
        policy_loss = []
        for log_prob, disc_return in zip(saved_log_probs, returns):
            policy_loss.append(-log_prob * disc_return)
        policy_loss = torch.stack(policy_loss).sum()

        optimizer.zero_grad()
        policy_loss.backward()
        optimizer.step()
        avg_score = np.mean(scores_deque)
        avg_scores.append(avg_score)  # Append average score for plotting
        if i_episode % print_every == 0:
            print('Episode {}\tAverage Score: {:.2f}'.format(i_episode, avg_score))
    return scores, avg_scores

def evaluate_agent(env, max_steps, n_eval_episodes, policy):
    episode_rewards = []
    for episode in range(n_eval_episodes):
        mj.mj_resetData(env.model, env.data)
        state = np.concatenate([env.data.qpos, env.data.qvel])
        total_rewards_ep = 0
        for step in range(max_steps):
            action, _ = policy.act(state)
            env.data.ctrl[:] = action
            mj.mj_step(env.model, env.data)
            state = np.concatenate([env.data.qpos, env.data.qvel])
            reward = compute_reward(state) 
            total_rewards_ep += reward
            if env.data.time >= simend:
                break
        episode_rewards.append(total_rewards_ep)
    mean_reward = np.mean(episode_rewards)
    std_reward = np.std(episode_rewards)
    return mean_reward, std_reward

def compute_reward(state):

    desired_state = np.zeros_like(state)
    reward = -np.sum((state - desired_state) ** 2)
    return reward

class MuJoCoEnv:
    def __init__(self, model, data):
        self.model = model
        self.data = data

    def reset(self):
        mj.mj_resetData(self.model, self.data)
        return self.data

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

# Initialize environment
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)
env = MuJoCoEnv(model, data)

state_size = model.nq + model.nv
action_size = model.nu

hyperparameters = {
    "h_size": 32,  # Increased hidden layer size
    "n_training_episodes": 2000,  # Increased number of episodes
    "n_evaluation_episodes": 10,
    "max_t": 200,
    "gamma": 0.99,
    "lr": 0.0005,  # Reduced learning rate
    "state_space": state_size,
    "action_space": action_size,
}

policy = Policy(hyperparameters["state_space"], hyperparameters["action_space"], hyperparameters["h_size"]).to(device)
optimizer = optim.Adam(policy.parameters(), lr=hyperparameters["lr"])

# GLFW setup
glfw.init()
window = glfw.create_window(1200, 900, "RL Control Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

cam = mj.MjvCamera()
opt = mj.MjvOption()
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

# Run training
scores, avg_scores = reinforce(policy, optimizer, hyperparameters["n_training_episodes"], hyperparameters["max_t"], hyperparameters["gamma"], 25)
torch.save(policy.state_dict(), "Rotary_Inverted_Pendulum.pt")

policy.load_state_dict(torch.load("Rotary_Inverted_Pendulum.pt"))
policy.to(device)
mean_reward, std_reward = evaluate_agent(env, hyperparameters["max_t"], hyperparameters["n_evaluation_episodes"], policy)
print(f"eval mean reward {mean_reward}  std reward {std_reward}")

plt.plot(avg_scores)
plt.xlabel('Episode')
plt.ylabel('Average Score')
plt.title('Training Progress')
plt.show()

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