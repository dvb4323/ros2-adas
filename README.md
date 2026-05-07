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
./CarlaUE4.sh -quality-level=Low -RenderOffScreen -nosound -windowed -ResX=320 -ResY=320 -benchmark -fps=20 --ros2
cd PythonAPI/examples
```

- Run the test script spawn_ros2_car.py which will spawn a car with autopilot on:

```bash
cd test/
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

### Package creation (In case you need a new one)

```bash
cd ~/ros2-adas/ros2-adas/src
ros2 pkg create lane_detection \
  --build-type ament_python \
  --dependencies rclpy sensor_msgs cv_bridge
cd ~/ros2-adas/ros2-adas
```

### Package build (Do after every script change)

With the actorID from the log of the spawn script, update the actorID in lane_detector.py:

```python
self.subscription = self.create_subscription(
            Image,
            '/carla/actor29/front_cam/image',  # <-- update this
            self.image_callback,
            10
        )
```

```bash
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

## Troubleshoot

### Port binded error

By default, CarlaUE4 uses port 2000, some services like Traffic Manager (TM) use script defined port (8005). When you stop the script, and then run again, there will be potential error since the last process run is still occupying the port.

Free the port by running:

```bash
sudo lsof -i:2000
kill -9 <PID>
```
