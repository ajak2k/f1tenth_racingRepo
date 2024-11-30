# Formulazot

**Formulazot** is a ROS2 package designed for autonomous racing applications, specifically for the F1Tenth platform. This package implements functionalities like wall following and integrates with other components through the `autodrive_f1tenth` package.  

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [License](#license)
- [Acknowledgments](#acknowledgments)

---

## Overview

The **Formulazot** package focuses on enabling autonomous vehicle navigation and behavior, such as:
- Wall following for obstacle avoidance.
- Integration with other nodes to create a seamless autonomous driving stack.

This package is used in the context of the **F1Tenth Autonomous Racing** framework.

---

## Features

- **Wall Following Node**: Implements a simple algorithm to follow walls for navigation.
- **ROS2 Integration**: Designed to work with ROS2 nodes and the F1Tenth stack.
- **Compatibility**: Works alongside `autodrive_f1tenth` for communication and data sharing.

---

## Installation

### Prerequisites
Ensure you have the following installed:
- ROS2 (Humble or later)
- Colcon build tools
- A working F1Tenth simulation environment or hardware setup

### Build Instructions
1. Clone the repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> formulazot
2. Navigate to your workspace and build the package:
    ```bash
    cd ~/ros2_ws
    colcon build
3. Source your workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash

---

## Usage
To launch the Formulazot nodes, use the provided launch file:
    ```bash
    ros2 launch formulazot wall_following.launch.py
### Launch File Details
The launch file includes:
- **autodrive_bridge**: Integrates the Formulazot package with the Autodrive F1Tenth stack.
- **wall_follower**: Executes the wall-following node.

## License
This package is licensed under the Apache License 2.0. See the **LICENSE** file for more details.

## Acknowledgments
This package is based on skeleton code by Hongrui (Billy) Zheng from the University of Pennsylvania as part of the ESE6150: F1Tenth Autonomous Racing Cars course.

@inproceedings{okelly2020f1tenth,
  title={F1TENTH: An Open-source Evaluation Environment for Continuous Control and Reinforcement Learning},
  author={O’Kelly, Matthew and Zheng, Hongrui and Karthik, Dhruv and Mangharam, Rahul},
  booktitle={NeurIPS 2019 Competition and Demonstration Track},
  pages={77--89},
  year={2020},
  organization={PMLR}
}

@inproceedings{AutoDRIVE-Simulator-2021,
author = {Samak, Tanmay Vilas and Samak, Chinmay Vilas and Xie, Ming},
title = {AutoDRIVE Simulator: A Simulator for Scaled Autonomous Vehicle Research and Education},
year = {2021},
isbn = {9781450390453},
publisher = {Association for Computing Machinery},
address = {New York, NY, USA},
url = {https://doi.org/10.1145/3483845.3483846},
doi = {10.1145/3483845.3483846},
booktitle = {2021 2nd International Conference on Control, Robotics and Intelligent System},
pages = {1–5},
numpages = {5},
location = {Qingdao, China},
series = {CCRIS'21}
}

Copyright 2024 Ajay Shankar Sriram