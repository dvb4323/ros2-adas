# Ros2-ADAS Project

## Environment Requirements

- Python 3.12.3
- Ros2 Humble
- Carla 0.9.16
- Ubuntu 22.04 LTS

## Project Structure

### Packages List

- lane_controller
- lane_detection
- carla_msgs

### Recommended Tools

- ros2 bag, foxglove for debugging

## Setup Guide

### Prerequisition

- Ros2 installed (Humble for Ubuntu 22.04, Kilted for 24.04)
- Carla Downloaded

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

- Run the test script spawn_ros2_car.py which will spawn a car with autopilot feature (can be turned on/off). You can also modify the map it gets spawned on:

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

- With the actorID from the log of the spawn script, update the actorID in lane_detector.py:

```python
self.subscription = self.create_subscription(
            Image,
            '/carla/actor29/front_cam/image',  # <-- update this
            self.image_callback,
            10
        )
```

- Build package lane_detection:  

```bash
colcon build --packages-select lane_detection --parallel-workers 1
source install/setup.bash
ros2 run lane_detection lane_detector
```

- Build package lane_controller:

```bash
colcon build --packages-select lane_controller --parallel-workers 1
source install/setup.bash
ros2 run lane_controller controller
```

### Check for image processing result

```bash
eog /tmp/frame.jpg
eog /tmp/edges.jpg
eot /tmp/roi.jpg
```

### Lane Detection Pipeline

Explained in docs/Lane Detection Pipeline.pdf

## Testing and Debug

### Use ros2 bag to record a session, then playback:

```bash
ros2 bag record -o recording -s mcap /carla/actor147/front_cam/image /tf /lane_offset /carla/actor147/vehicle_control_cmd
ros2 bag play recording/rosbag2_2026_05_12-10_44_44_0.mcap
```

### Debug using Foxglove

- Create an account on Foxglove's homepage, then create a project
- Install Foxglove:

```bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
source /opt/ros/<Ros_Version>/setup.sh
source install/setup.sh
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

- Return to Foxglove Dashboard, open a connection to ws://localhost:8765 using Google Chrome

## Troubleshoot

### Port binded error

By default, CarlaUE4 uses port 2000, some services like Traffic Manager (TM) use script defined port (8005). When you stop the script, and then run again, there will be potential error since the last process run is still occupying the port.

Free the port by running:

```bash
sudo lsof -i:2000
kill -9 <PID>
```

## Note

- CARLA is resource intensive, is should be run on devices with external GPU (add flag -preferNvidia when run with a Nvidia supported pc)
- For Lane Keeping Assistant Testing, choose map with long straight road: Town4

### References

- [Ros2 Humble documentation](https://docs.ros.org/en/humble/index.html)
- [Carla Documentation](https://carla.readthedocs.io/en/latest/)
- [Foxglove Homepage](https://app.foxglove.dev/)
