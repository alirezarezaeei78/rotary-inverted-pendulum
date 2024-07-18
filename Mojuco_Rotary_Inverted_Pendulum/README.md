# MuJoCo Rotary Inverted Pendulum Model

## Overview

This folder contains the MuJoCo XML model for the rotary inverted pendulum system. The model defines the physical properties, geometry, and dynamics of the system, including joints, actuators, sensors, and visual elements.

---

## 📄 XML Model Description

The XML model defines the rotary inverted pendulum system, which consists of two main links connected by hinge joints, actuated by motors, and equipped with sensors to monitor the joint positions. Below is a detailed description of the components defined in the XML file.

### 📁 Components

1. **Visual Elements**: 
    - Defines the lighting, headlight properties, and global visual settings.
    - Adds a gradient skybox texture and a checkerboard texture for the ground plane.

2. **Assets**: 
    - Contains textures and materials used for visualizing the environment and the pendulum.

3. **Options**: 
    - Sets the global simulation options such as gravity.

4. **World Body**: 
    - Defines the environment and the physical setup of the rotary inverted pendulum.
    - Includes a stand and two links connected by hinge joints.

5. **Actuators**: 
    - Defines the motors that apply torque to the joints to control the pendulum's movement.

6. **Sensors**: 
    - Includes sensors to measure the position of the joints.

---

## 📐 Model Details

### Visual Elements
- **Headlight**: Provides diffuse and ambient lighting with specified intensity.
- **Skybox**: A gradient texture is used to create a skybox effect.
- **Ground Plane**: A checkerboard texture is applied to the ground for better visual reference.

### Assets
- **Textures**: 
  - Skybox texture: Gradient from `rgb1="0.3 0.5 0.7"` to `rgb2="0 0 0"`.
  - Groundplane texture: Checker pattern with colors `rgb1="0.2 0.3 0.4"` and `rgb2="0.1 0.2 0.3"`.

### Options
- **Gravity**: Set to `0 0 -9.81` to simulate Earth's gravity.

### World Body
- **Light**: Directional light positioned at `0 0 1.5`.
- **Floor**: Plane geometry for the ground with a checkerboard texture.
- **Stand**: Cylinder geometry acting as a stand for the pendulum.
- **First Link**: Cylinder geometry representing the first link of the pendulum, connected by `joint0`.
- **Second Link**: Cylinder geometry representing the second link of the pendulum, connected by `joint1`.

### Actuators
- **Motors**: 
  - `torque0`: Applies torque to `joint0`.
  - `torque1`: Applies torque to `joint1`.

### Sensors
- **Joint Position Sensors**: 
  - `j1`: Measures the position of `joint0`.
  - `j2`: Measures the position of `joint1`.

---

## 🚀 Getting Started

To use this MuJoCo model, follow these steps:

1. **Clone the Repository**: Clone this repository to your local machine using the following command:
    ```bash
    git clone https://github.com/alirezarezaeei78/rotary-inverted-pendulum.git
    cd Mojuco_Rotary_Inverted_Pendulum
    ```

2. **Set Up MuJoCo**: Ensure you have MuJoCo installed and properly set up on your system. You may need to obtain a MuJoCo license and follow the installation instructions provided on the [MuJoCo website](http://www.mujoco.org/).

3. **Load the Model**: Load the XML model into your MuJoCo environment using your preferred method (e.g., via a Python script or the MuJoCo GUI).

---

## 📞 Contact

If you have questions or need assistance, you can reach out to the project maintainer:

Email: [alireza78@email.kntu.ac.ir](mailto:alireza78@email.kntu.ac.ir)
