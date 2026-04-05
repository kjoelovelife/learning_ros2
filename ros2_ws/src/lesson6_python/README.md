# lesson6_python

Python service server and client to control turtlesim movement paths via `lesson_interfaces/MoveTurtlesim`.

## Dependencies

Requires `lesson_interfaces` to be built first.

## Nodes

| Executable | Node class | Description |
|------------|-----------|-------------|
| `move_turtlesim_server_exe` | `MoveTurtlesimServer` | Service server that drives the turtle through the requested path by publishing `Twist` to `/turtle1/cmd_vel` |
| `move_turtlesim_client_exe` | `MoveTurtlesomClient` | Service client that requests all four paths sequentially |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/move_turtlesim` | `lesson_interfaces/MoveTurtlesim` | Request `path` string; response `bool successful` |

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/turtle1/cmd_vel` | `geometry_msgs/Twist` | Server → turtlesim |

## Supported paths

`"line"`, `"square"`, `"circle"`, `"triangle"`

## Usage

```bash
# Terminal 1 – start turtlesim
ros2 run turtlesim turtlesim_node

# Or use the launch file (starts server + client together)
ros2 launch lesson6_python move_turtlesim.launch.py
```
