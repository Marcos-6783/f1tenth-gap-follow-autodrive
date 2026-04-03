# F1TENTH Follow the Gap - AutoDRIVE

This repository contains the implementation of a reactive autonomous navigation algorithm (Follow the Gap) using ROS 2 Humble and the AutoDRIVE Simulator.

## Overview
The node processes LiDAR data to navigate a vehicle through a track by identifying the largest safe "gap" in the environment. It includes a safety bubble to avoid collisions with walls and obstacles.

## Requirements
* **OS:** Ubuntu 22.04
* **Middleware:** ROS 2 Humble
* **Simulator:** AutoDRIVE (Unity-based)
* **Language:** C++ / Python 3.10

## Installation
1. Clone this repository into your workspace:
   ```bash
   cd ~/autodrive_ws/src
   git clone [https://github.com/Marcos-6783/f1tenth-gap-follow-autodrive.git](https://github.com/Marcos-6783/f1tenth-gap-follow-autodrive.git)
