# Ros2-ADAS Project

## Environment Requirements

- Python 3.12.3
- Ros2 Humble
- Carla 0.9.16
- Ubuntu 22.04 LTS

## Setup Guide

### Install necessary packages

```bash
export PIP_BREAK_SYSTEM_PACKAGES=1
pip install ~/CARLA_0.9.16/PythonAPI/carla/dist/carla-0.9.16-*.whl
```

### Install Carla examples library

```bash
cd /
cd home/Username/CARLA_0.9.16/PythonAPI/examples
python3 -m pip install -r requirements.txt
```

### Install Ros Bridge (Legacy, not needed since CARLA 0.9.16 has built-in ros bridge)

```bash
git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git src/ros-bridge
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build \
  --packages-up-to carla_ros_bridge \
  --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Verifying Installation

```bash
cd ~/CARLA_0.9.16
./CarlaUE4.sh -quality-level=Low -RenderOffScreen -nosound -windowed -ResX=32 -ResY=32 -benchmark -fps=10 --ros2
cd PythonAPI/examples
```

- Copy the test script spawn_ros2_car.py to examples/

```bash
python3 spawn_ros2_car.py
ros2 topic list
```

- Now, you should see something like this in the other terminal, which list the topics:

```bash
/carla/actor49/ackermann_control_cmd
/carla/actor49/vehicle_control_cmd
/clock
/parameter_events
/rosout
/tf
```

### Package creation

```bash
cd ~/ros2-adas/ros2-adas/src
ros2 pkg create lane_detection \
  --build-type ament_python \
  --dependencies rclpy sensor_msgs cv_bridge
cd ~/ros2-adas/ros2-adas
colcon build --packages-select lane_detection --parallel-workers 1
source install/setup.bash
ros2 run lane_detection lane_detector
```

### Check for image processing result

```bash
eog /tmp/frame.jpg
eog /tmp/edges.jpg
eot /tmp/roi.jpg
```

### Lane Detection Pipeline

Explained in docs/Lane Detection Pipeline.pdf

