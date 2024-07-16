import gymnasium as gym
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

# PyTorch
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Normal

# MuJoCo
import mujoco as mj
from mujoco.glfw import glfw
import os
import control

# Set device
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print(f"Device: {device}")

### RL Controller ###

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
        state = env.reset()[0]
        for t in range(max_t):
            action, log_prob = policy.act(state)
            saved_log_probs.append(log_prob)
            state, reward, done, _, _ = env.step(action)
            rewards.append(reward)
            if done:
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

### PID Controller ###

Kp = 100.0
Ki = 1.0
Kd = 20.0
integral = np.zeros(2)
previous_error = np.zeros(2)

def pid_control(target, current, dt):
    global integral, previous_error
    error = target - current
    integral += error * dt
    derivative = (error - previous_error) / dt
    previous_error = error
    return Kp * error + Ki * integral + Kd * derivative

def pid_controller(model, data):
    target = np.array([0.0, 0.0])  # Target positions for the joints
    current = np.array([data.qpos[0], data.qpos[1]])
    dt = model.opt.timestep
    control_signal = pid_control(target, current, dt)
    data.ctrl[0] = control_signal[0]
    data.ctrl[1] = control_signal[1]

    # Apply disturbance torque
    tau_disturb_mean = 0
    tau_disturb_dev = 150
    tau_d0 = np.random.normal(tau_disturb_mean, tau_disturb_dev)
    tau_d1 = np.random.normal(tau_disturb_mean, 0.25 * tau_disturb_dev)
    data.qfrc_applied[0] = tau_d0
    data.qfrc_applied[1] = tau_d1

### LQR Controller ###

def f(x, u):
    data.qpos[0] = x[0]
    data.qpos[1] = x[1]
    data.qvel[0] = x[2]
    data.qvel[1] = x[3]
    data.ctrl[0] = u[0]
    mj.mj_forward(model, data)

    M = np.zeros((2, 2))
    mj.mj_fullM(model, M, data.qM)
    invM = np.linalg.inv(M)
    frc_bias = np.array([data.qfrc_bias[0], data.qfrc_bias[1]])
    tau = np.array([u[0], 0])
    qddot = np.matmul(invM, np.subtract(tau, frc_bias))

    xdot = np.array([data.qvel[0], data.qvel[1], qddot[0], qddot[1]])
    return xdot

def linearize():
    n = 4
    m = 1
    A = np.zeros((n, n))
    B = np.zeros((n, m))

    x0 = np.array([0, 0, 0, 0])
    u0 = np.array([0])
    xdot0 = f(x0, u0)

    pert = 1e-2
    for i in range(0, n):
        x = np.copy(x0)
        x[i] += pert
        xdot = f(x, u0)
        A[:, i] = (xdot - xdot0) / pert

    for i in range(0, m):
        u = np.copy(u0)
        u[i] += pert
        xdot = f(x0, u)
        B[:, i] = (xdot - xdot0) / pert

    return A, B

def init_lqr_controller():
    global K
    A, B = linearize()

    print("A matrix:\n", A)
    print("B matrix:\n", B)

    Q = np.eye(4)
    R = np.eye(1) * 1e-2
    try:
        K, _, _ = control.lqr(A, B, Q, R)
        print("LQR controller gain K:\n", K)
    except np.linalg.LinAlgError as e:
        print("LQR failed to find a finite solution:", e)
        K = np.zeros((1, 4))  # Default to zero gain if LQR fails

def lqr_controller(model, data):
    global K
    x = np.array([data.qpos[0], data.qpos[1], data.qvel[0], data.qvel[1]])
    u = -K @ x
    data.ctrl[0] = u[0]

    # Apply disturbance torque
    tau_disturb_mean = 0
    tau_disturb_dev = 20
    tau_d0 = np.random.normal(tau_disturb_mean, tau_disturb_dev)
    tau_d1 = np.random.normal(tau_disturb_mean, 0.25 * tau_disturb_dev)
    data.qfrc_applied[0] = tau_d0
    data.qfrc_applied[1] = tau_d1

### Main Script for Comparison ###

def run_controller(controller_func, episodes=100, max_steps=200):
    rewards = []
    for episode in range(episodes):
        mj.mj_resetData(model, data)
        total_reward = 0
        for step in range(max_steps):
            controller_func(model, data)
            mj.mj_step(model, data)
            reward = -np.square(data.qpos).sum() - 0.1 * np.square(data.qvel).sum()  # Example reward function
            total_reward += reward
            if data.time >= simend:
                break
        rewards.append(total_reward)
    return rewards

def plot_performance(rl_scores, pid_scores, lqr_scores):
    plt.plot(rl_scores, label='RL')
    plt.plot(pid_scores, label='PID')
    plt.plot(lqr_scores, label='LQR')
    plt.xlabel('Episode')
    plt.ylabel('Total Reward')
    plt.title('Controller Performance Comparison')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    # MuJoCo model and simulation setup
    xml_path = '2D_double_pendulum.xml'
    simend = 15  # Simulation end time

    dirname = os.path.dirname(__file__)
    abspath = os.path.join(dirname, xml_path)
    model = mj.MjModel.from_xml_path(abspath)
    data = mj.MjData(model)
    cam = mj.MjvCamera()
    opt = mj.MjvOption()

    glfw.init()
    window = glfw.create_window(1200, 900, "Controller Comparison", None, None)
    glfw.make_context_current(window)
    glfw.swap_interval(1)
    scene = mj.MjvScene(model, maxgeom=10000)
    context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

    cam.azimuth = 35
    cam.elevation = -15
    cam.distance = 7
    cam.lookat = np.array([2.0, 1.5, 1.0])

    env_id = "Pendulum-v1"
    env = gym.make(env_id)

    state_size = env.observation_space.shape[0]
    action_size = env.action_space.shape[0]

    # RL Controller
    rl_hyperparameters = {
        "h_size": 32,
        "n_training_episodes": 1000,
        "n_evaluation_episodes": 10,
        "max_t": 200,
        "gamma": 0.99,
        "lr": 0.001,
        "env_id": env_id,
        "state_space": state_size,
        "action_space": action_size,
    }

    rl_policy = Policy(rl_hyperparameters["state_space"], rl_hyperparameters["action_space"], rl_hyperparameters["h_size"]).to(device)
    optimizer = optim.Adam(rl_policy.parameters(), lr=rl_hyperparameters["lr"])
    rl_scores, avg_scores = reinforce(rl_policy, optimizer, rl_hyperparameters["n_training_episodes"], rl_hyperparameters["max_t"], rl_hyperparameters["gamma"], 25)
    torch.save(rl_policy.state_dict(), "double_pendulum_reinforce.pt")
    rl_policy.load_state_dict(torch.load("double_pendulum_reinforce.pt"))
    rl_policy.to(device)

    # Initialize and run controllers
    mj.set_mjcb_control(pid_controller)
    pid_scores = run_controller(pid_controller, episodes=100)

    init_lqr_controller()
    mj.set_mjcb_control(lqr_controller)
    lqr_scores = run_controller(lqr_controller, episodes=100)

    plot_performance(avg_scores, pid_scores, lqr_scores)
    glfw.terminate()
