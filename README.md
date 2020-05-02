# ROS2 LGSVL Bridge

This is bridge that exposes custom TCP socket for accepting and transmitting
ROS2 messages from LGSVL Simulator.

# Requirements

* rcutils - https://github.com/ros2/rcutils
* rcl - https://github.com/ros2/rcl
* boost (with asio) - https://www.boost.org/

# Building

```
colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Release'
```

# Running

```
. ~/ros2-lgsvl-bridge/install/setup.bash
lgsvl_bridge [--port 9090] [--log D]
```
