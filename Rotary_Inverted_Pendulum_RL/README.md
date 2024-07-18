# Rotary Inverted Pendulum - Reinforcement Learning Control

## Overview

This project implements a Reinforcement Learning (RL) control system for a rotary inverted pendulum using the MuJoCo physics engine and PyTorch for the neural network. The objective is to stabilize the pendulum in its upright position through RL techniques.

---

## 📘 Project Structure

This project is structured into several key sections to facilitate understanding and replication of the results. Below is a detailed breakdown of each section:

### 📂 Files

- `Rotary_Inverted_Pendulum_RL.py`: Main Python script implementing the RL controller.

---

## 🚀 Getting Started

### Prerequisites

Ensure you have the following software installed on your machine:

- Python 3.x
- MuJoCo
- PyTorch
- OpenGL (for GLFW)

### Installation

 **Clone the repository**:

    ```bash
    git clone https://github.com//alirezarezaeei78/rotary-inverted-pendulum.git
    cd Rotary_Inverted_Pendulum_RL
    ```


---

## 🎮 Running the RL Controller

### Description

The RL controller is implemented using a neural network policy with the REINFORCE algorithm. The MuJoCo environment simulates the rotary inverted pendulum, and the RL agent learns to stabilize it through episodes of interaction.

### Key Components

1. **Policy Network**: 
    - A neural network with two hidden layers used to output the mean and standard deviation of the action distribution.

2. **REINFORCE Algorithm**:
    - A policy gradient method used to train the policy network based on the rewards obtained from the environment.

3. **Environment**:
    - Simulated using MuJoCo, which provides the dynamics of the rotary inverted pendulum.

### Training

The training process involves running multiple episodes where the agent interacts with the environment, collects rewards, and updates the policy network.

### Evaluation

After training, the policy is evaluated over several episodes to measure its performance in stabilizing the pendulum.

---

## 📊 Results

- **Training Progress**: The script plots the average score over episodes, showing the learning progress of the RL agent.
- **Evaluation Metrics**: The mean and standard deviation of rewards during evaluation episodes indicate the performance and stability of the trained policy.

---

## 🛠️ Code Explanation

Below is a high-level overview of the main sections of the RL control script:

- **Imports and Setup**: Import necessary libraries and set up the MuJoCo environment and PyTorch device.
- **Policy Class**: Defines the neural network architecture for the policy.
- **REINFORCE Function**: Implements the training loop for the RL agent.
- **Evaluation Function**: Evaluates the trained policy over multiple episodes.
- **Main Script**:
    - Initializes the environment and policy.
    - Runs the training loop.
    - Evaluates the trained policy.
    - Plots the training progress.
    - Runs the simulation visualization.

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 📞 Contact

If you have any questions or feedback, please feel free to contact:

- **Email**: alireza78@email.kntu.ac.ir
- **GitHub**: [alirezarezaeei78](https://github.com/alirezarezaeei78)

---

## 📚 References

1. [MuJoCo Documentation](https://mujoco.readthedocs.io/)
2. [PyTorch Documentation](https://pytorch.org/docs/stable/index.html)
3. [Reinforcement Learning: An Introduction](http://incompleteideas.net/book/RLbook2018.pdf) by Sutton and Barto

