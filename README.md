# F1TENTH Autonomous Navigation & Multimodal Anomaly Detection (AutoDRIVE)

## Project Description
This project implements a reactive autonomous navigation system for an F1TENTH vehicle using the AutoDRIVE ecosystem and ROS 2 Humble. The core of the system is based on the Follow the Gap algorithm, which allows the vehicle to avoid obstacles at high speeds by searching for the deepest space within the LiDAR horizon. Additionally, it integrates a multimodal anomaly detection layer that cross-references laser perception data with the vehicle's dynamic telemetry.

### Reactive Planning – Follow the Gap
A control node is implemented to process LiDAR point clouds to identify safe "gaps". The algorithm applies a Safety Bubble around detected obstacles to prevent collisions and selects the center of the deepest gap as the steering target.

## 1. Installation

### 1.0 Install ROS-based dependencies:
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip python3-rosdep python3-colcon-common-extensions git
sudo apt install ros-humble-ackermann-msgs
sudo apt install ros-humble-nav2-msgs
```
### 1.1 Clone and install all dependencies:
```bash
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update

mkdir -p ~/autodrive_ws/src
cd ~/autodrive_ws/src
# Clone the AutoDRIVE Bridge
git clone [https://github.com/Triton-AI/autodrive_ros2.git](https://github.com/Triton-AI/autodrive_ros2.git)
# Clone this repository
git clone [https://github.com/Marcos-6783/f1tenth-gap-follow-autodrive.git](https://github.com/Marcos-6783/f1tenth-gap-follow-autodrive.git)

cd ~/autodrive_ws
rosdep install --from-paths src --ignore-src -r -y
```
### 1.2 Build your workspace:
```bash
cd ~/autodrive_ws
colcon build --packages-select gap_follow autodrive_msgs
source install/setup.bash
```
### Estructura del paquete ROS 2
```bash
autodrive_ws/
└── src/
    ├── f1tenth-gap-follow-autodrive/
    │   ├── gap_follow/
    │   │   ├── src/
    │   │   │   └── reactive_node.cpp
    │   │   ├── launch/
    │   │   └── CMakeLists.txt
    └── autodrive_ros2/
        └── autodrive_bridge/
```
### ROS Node Graph
    1. Reactive Node: Processes LiDAR data and calculates steering/throttle.

    2. AutoDRIVE Bridge: Connects ROS 2 with the Unity simulator.

    3. Simulator: Virtual testing environment.

    4. RViz2: Visualization of LaserScan data and Safety Bubbles.
    
### Launch files & Execution
Running the Project
Bridge:
```bash
source ~/autodrive_ws/install/setup.bash
ros2 launch autodrive_ros2 autodrive_bridge.launch.py
Reactive Node: Procesa LiDAR y calcula steering/throttle.
```
Reactive Navigation:
```bash
source ~/autodrive_ws/install/setup.bash
ros2 run gap_follow reactive_node
```

### ⚠️ Multimodal Anomaly Detection
The system integrates advanced safety logic:

Spatial Anomaly (LiDAR): Identifies unmapped obstacles or sudden changes in the environment.

State Anomaly (Telemetry): The /telemetry topic is monitored. If the acceleration command (throttle) is greater than zero but the speed reported by the encoders is zero, the system detects a stall or loss of traction and activates an emergency stop protocol.

### Project Execution
Results:https://drive.google.com/file/d/1-0ueHXua01EAlZdTATlNUiUPNk7Jf6PK/view?usp=sharing

### 📊 Visual Results

<img width="1276" height="711" alt="image" src="https://github.com/user-attachments/assets/3ed9340b-c423-4427-925c-d80858181082" />
<img width="1366" height="768" alt="image" src="https://github.com/user-attachments/assets/5de64e89-5c16-4a77-8a65-5f121de7eb67" />

Note: Tests were conducted in the AutoDRIVE testbed, achieving stable navigation without collisions by mitigating noise in the LiDAR sensor.

### Code Explanation
reactive_node.cpp (Follow the Gap Implementation)
The node subscribes to:

/autodrive/f1tenth_1/lidar (sensor_msgs/msg/LaserScan)

And publishes:

/autodrive/f1tenth_1/steering_command (ackermann_msgs/msg/AckermannDriveStamped)

/autodrive/f1tenth_1/throttle_command

Processing Logic:

Preprocessing: LiDAR data is filtered to remove infinite values and noise.

Find Closest Point: Locates the obstacle nearest to the vehicle.

Eliminate Bubble: "Draws" a safety radius (zeros) around that point to prevent the car from attempting to pass too closely.

Find Best Gap: Searches for the longest sequence of points with maximum distances.

Steering Calculation: Targets the center of the selected gap.

### Author
Marcos Emmanuel Balón García - Escuela Superior Politécnica del Litoral (ESPOL)

Autonomous Robotics Project - ROS 2
