# lesson7_python

Demonstrates ROS 2 namespaces, topic remapping, and keyboard teleoperation with turtlesim.

## Nodes

| Executable | Node class | Description |
|------------|-----------|-------------|
| `teleop_in_terminal_exe` | `TeleopInTerminal` | Keyboard teleop; reads raw keystrokes and publishes `Twist` to `/turtle1/cmd_vel` |

## Controls (teleop_in_terminal)

```
    w
a   s   d
    x

w/x : forward / backward
a/d : turn left / right
s   : stop
Ctrl+C : quit
```

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/turtle1/cmd_vel` | `geometry_msgs/Twist` | TeleopInTerminal → turtlesim |

## Launch files

| File | Description |
|------|-------------|
| `teleop_turtlesim_namespace.launch.py` | Demonstrates launching nodes under a namespace |
| `teleop_turtlesim_remap.launch.py` | Demonstrates topic remapping between nodes |
| `teleop_green_sea_turtle.launch.py` | Exercise: wire up teleop to a custom turtle name |
| `answer/teleop_green_sea_turtle_answer.launch.py` | Reference answer for the exercise |

## Usage

```bash
# Terminal 1 – start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2 – start keyboard teleop
ros2 run lesson7_python teleop_in_terminal_exe
```
