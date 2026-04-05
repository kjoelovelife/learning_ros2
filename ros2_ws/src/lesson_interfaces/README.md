# lesson_interfaces

Custom ROS 2 message and service definitions shared across all lesson packages.

## Interfaces

### Messages

| File | Fields | Used by |
|------|--------|---------|
| `msg/Lunch.msg` | `int8 bowls_of_rice`, `std_msgs/ColorRGBA color_of_bowls`, `string[] vegetables`, `string[] meats` | lesson5 |
| `msg/Turtle.msg` | `name`, `pose` | lesson8 |
| `msg/TurtleArray.msg` | `Turtle[] turtle_array` | lesson8 |

### Services

| File | Request | Response | Used by |
|------|---------|----------|---------|
| `srv/MoveTurtlesim.srv` | `string path` | `bool successful` | lesson6 |
| `srv/CatchTurtle.srv` | `string name` | `bool success` | lesson8 |

## Build

This package must be built before any lesson package that depends on it:

```bash
cd ros2_ws
colcon build --packages-select lesson_interfaces
source install/setup.bash
```
