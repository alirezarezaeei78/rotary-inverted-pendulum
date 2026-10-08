# Rotary Inverted Pendulum Control

Python experiments for controlling a rotary inverted pendulum in **MuJoCo**, with classical controllers and a PyTorch reinforcement-learning policy.

The repository brings together the simulation model, controller scripts, plots, and recorded demonstrations. It is an educational control project, rather than a validated hardware controller or standardized controller benchmark.

## Controllers

| Approach | Implementation |
| --- | --- |
| PID | [PID controller](Rotary_Inverted_Pendulum_PID/Rotary_Inverted_Pendulum_PID.py) |
| LQR | [Linear quadratic regulator](Rotary_Inverted_Pendulum_LQR/Rotary_Inverted_Pendulum_LQR.py) |
| Reinforcement learning | [REINFORCE policy-gradient experiment](Rotary_Inverted_Pendulum_RL/Rotary_Inverted_Pendulum_RL.py), using a Gaussian policy in PyTorch |

The RL script trains a continuous-action policy and saves a model checkpoint. Its learning curve is an experimental result; it does not establish a general performance advantage over PID or LQR.

## Setup

Use Python with a desktop graphics environment capable of running MuJoCo's GLFW viewer.

```sh
git clone https://github.com/alirezarezaeei78/rotary-inverted-pendulum.git
cd rotary-inverted-pendulum
python -m venv .venv
```

Activate the environment (`.venv\Scripts\activate` on Windows, or `source .venv/bin/activate` on macOS/Linux), then install the dependencies:

```sh
python -m pip install -r requirements.txt
```

Run one controller at a time from the repository root:

```sh
python Rotary_Inverted_Pendulum_PID/Rotary_Inverted_Pendulum_PID.py
python Rotary_Inverted_Pendulum_LQR/Rotary_Inverted_Pendulum_LQR.py
python Rotary_Inverted_Pendulum_RL/Rotary_Inverted_Pendulum_RL.py
```

The scripts resolve the shared XML model relative to their own file locations. Review simulation duration, gains, and training settings before running a script. RL training can take substantially longer than a classical-controller demonstration.

## Model and results

- [MuJoCo model](Mojuco_Rotary_Inverted_Pendulum/rotary%20inverted%20pendulum.xml)
- [Plots and figures](Img/)
- [PID demonstration](Rotary_Inverted_Pendulum_control_videos/rotary%20inverted%20pendulum_PID.movie.mp4)
- [LQR demonstration](Rotary_Inverted_Pendulum_control_videos/rotary%20inverted%20pendulum%20lqr.mp4)
- [Supporting literature](Articles/)

Existing folder names are retained so repository links continue to work.

## Author

[Alireza Rezaei](https://www.linkedin.com/in/alireza-rezaei-24963a210/) — electrical engineering, control systems, and applied machine learning.
