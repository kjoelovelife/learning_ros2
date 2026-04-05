# learning_ros2

A progressive ROS 2 tutorial series with parallel C++ (`rclcpp`) and Python (`rclpy`) implementations. Each lesson builds on the previous, covering core ROS 2 concepts from basic pub/sub through sensor data processing.

## Requirements

- ROS 2 Foxy
- Python 3.8+
- C++14

## Workspace setup

```bash
cd ros2_ws
source /opt/ros/foxy/setup.bash
colcon build
source install/setup.bash
```

## Lessons

| Lesson | Topic | Packages |
|--------|-------|---------|
| 4 | Publisher / Subscriber | `lesson4_cmake`, `lesson4_python` |
| 5 | Custom message types | `lesson5_cmake`, `lesson5_python` |
| 6 | Services (server & client) | `lesson6_cmake`, `lesson6_python` |
| 7 | Namespaces, remapping & teleop | `lesson7_python` |
| 8 | Multi-node system (turtle game) | `lesson8_python` |
| 9 | Parameters (declare, get, set) | `lesson9_python` |
| 10 | Sensor data & RViz | `lesson10_python` |

Each package has its own `README.md` with node descriptions, topic/service tables, and usage examples.

## Custom interfaces

`lesson_interfaces` defines the shared messages and services used across lessons 5–10. Build it first if building packages individually:

```bash
colcon build --packages-select lesson_interfaces
source install/setup.bash
```

## Running a lesson

```bash
# Example: lesson 8 turtle game
ros2 run turtlesim turtlesim_node &
ros2 launch lesson8_python catch_the_turtle.launch.py
```

## License

Apache-2.0 — see [LICENSE](LICENSE).

## Maintainer

Weichih Lin — weichih.lin@protonmail.com
