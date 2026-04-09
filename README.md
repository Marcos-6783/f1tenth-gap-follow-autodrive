# F1TENTH Autonomous Navigation & Multimodal Anomaly Detection (AutoDRIVE)

## Descripción del proyecto
Este proyecto implementa un sistema de navegación autónoma reactiva para un vehículo **F1TENTH** utilizando el ecosistema **AutoDRIVE** y **ROS 2 Humble**. El núcleo del sistema se basa en el algoritmo **Follow the Gap**, el cual permite al vehículo esquivar obstáculos a altas velocidades buscando el espacio más profundo en el horizonte del LiDAR. Además, se integra una capa de **detección de anomalías multimodal** que cruza datos de percepción láser con telemetría dinámica del vehículo.

### Planificación Reactiva – Follow the Gap
Se implementa un nodo de control que procesa las nubes de puntos del LiDAR para identificar "huecos" seguros. El algoritmo aplica una **Safety Bubble** (burbuja de seguridad) alrededor de los obstáculos detectados para prevenir colisiones y selecciona el centro del gap más profundo como objetivo de dirección.

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

Reactive Node: Procesa LiDAR y calcula steering/throttle.

AutoDRIVE Bridge: Conecta ROS 2 con el simulador Unity.

Simulator: Entorno virtual de pruebas.

RViz2: Visualización de LaserScan y Safety Bubbles.
