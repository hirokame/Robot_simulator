# 🤖 Robot Simulator Playground

Welcome to the **Robot Simulator Playground**! This repository is a collection of robotics simulation demos, ranging from a simple 2D "Robot Kata" to industry-standard 3D physics engines like PyBullet, MuJoCo, and Gazebo with ROS 2.

Whether you're just starting with robotics or looking for a quick reference for different simulators, this project provides a hands-on quick-start for each platform.

---

## 🚀 Quick Start Guide

Check out [ROBOTICS_SIMULATORS_GUIDE.md](./ROBOTICS_SIMULATORS_GUIDE.md) for a side-by-side comparison of the simulators included in this repo.

### 1. Simple 2D Robot Kata
A grid-based simulator useful for learning basic logic, command execution, and unit testing.

- **File:** `robot_simulator.py`
- **Run:** `python robot_simulator.py`

### 2. PyBullet
Lightweight and fast, great for Reinforcement Learning (RL) and quick prototyping.

- **File:** `demo_pybullet.py`
- **Install:** `pip install pybullet`
- **Run:** `python demo_pybullet.py`
- **Demo:** Loads a KUKA iiwa robot arm and simulates a falling ball.

### 3. MuJoCo
The #1 simulator for robotics research, known for its extremely accurate contact physics.

- **File:** `demo_mujoco.py`
- **Install:** `pip install mujoco`
- **Run:** `python demo_mujoco.py`
- **Demo:** Simulates a 2-link robotic arm defined entirely in an XML string.

### 4. Gazebo + ROS 2
Full robot system simulation used in industry, involving the complete ROS 2 stack.

- **File:** `demo_gazebo_ros2.py`
- **Prerequisites:** Ubuntu Linux with ROS 2 Humble and Gazebo installed.
- **Run:** Follow terminal instructions in the script or guide.
- **Demo:** Drives a TurtleBot3 in a square pattern with basic obstacle avoidance.

---

## 🛠️ Installation

Depending on which simulator you want to explore, you may need some or all of the following:

```bash
# For PyBullet
pip install pybullet

# For MuJoCo
pip install mujoco numpy

# For ROS 2 / Gazebo
# (Requires Ubuntu 22.04 + ROS 2 Humble)
sudo apt install ros-humble-ros-gz ros-humble-turtlebot3*
```

---

## 📂 Project Structure

- `robot_simulator.py`: Pure Python 2D logic.
- `demo_pybullet.py`: 3D simulation using Bullet physics.
- `demo_mujoco.py`: 3D simulation using MuJoCo physics.
- `demo_gazebo_ros2.py`: ROS 2 controller for TurtleBot3 in Gazebo.
- `ROBOTICS_SIMULATORS_GUIDE.md`: Comprehensive comparison and install guide.
- `robot_simulator_proof.md`: Documentation and testing proof for the 2D simulator.

---

## 💡 Which simulator should I use?

- **Use PyBullet** if you need something simple, fast to install, and good for RL.
- **Use MuJoCo** if you need high-fidelity manipulation or dexterous contact physics.
- **Use Gazebo + ROS 2** if you want to simulate a full robot system (navigation, sensors, etc.) as it would run on real hardware.

---

*Happy Simulating!* 🤖✨
