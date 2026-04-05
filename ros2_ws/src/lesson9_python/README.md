# lesson9_python

Demonstrates ROS 2 parameter declaration, getting, and setting — both from within a node and by calling the parameter services of another node.

## Nodes

| Executable | Node class | Description |
|------------|-----------|-------------|
| `turtle_draw_circle_exe` | `TurtleDrawCircle` | Declares a `velocity` parameter (range −1.0–1.0) and publishes `Twist` to `cmd_vel` at 2 Hz using it |
| `get_turtlesim_background_exe` | `GetTurtlesimBackground` | Calls turtlesim's `get_parameters` service every second to read `background_r/g/b` |
| `set_turtle_velocity_exe` | `SetTurtleVelocity` | Reads the `velocity` parameter descriptor from another node, then smoothly ramps velocity up/down at 20 Hz using `set_parameters_atomically` |
| `final_exam_exe` | `FinalExam` | Combines velocity ramp and dynamic background color change: sets turtlesim background R proportional to current velocity |

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | TurtleDrawCircle → turtlesim |

## Usage

```bash
# Terminal 1 – start turtlesim
ros2 run turtlesim turtlesim_node

# Draw a circle with configurable speed (set velocity via ros2 param set)
ros2 launch lesson9_python turtle_draw_circle_with_parameter.launch.py

# Read turtlesim background color parameters
ros2 launch lesson9_python get_turtlesim_background.launch.py

# Ramp turtle velocity dynamically
ros2 launch lesson9_python set_turtle_velocity.launch.py

# Final exam: velocity + dynamic background color
ros2 launch lesson9_python final_exam.launch.py
```

### Changing a parameter at runtime

```bash
ros2 param set /turtle_draw_circle velocity 0.5
```
