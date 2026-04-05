# lesson6_cmake

C++ service server and client to control turtlesim movement paths via `lesson_interfaces/MoveTurtlesim`.

## Dependencies

Requires `lesson_interfaces` to be built first.

## Nodes

| Executable | Node class | Description |
|------------|-----------|-------------|
| `move_turtlesim_server_exe` | `MoveTurtlesimServer` | Service server that moves the turtle in the requested path |
| `move_turtlesim_client_exe` | `MoveTurtlesimClient` | Service client that requests movement paths sequentially |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/move_turtlesim` | `lesson_interfaces/MoveTurtlesim` | Request `path` string; response `bool successful` |

## Supported paths

`"line"`, `"square"`, `"circle"`, `"triangle"`

## Usage

```bash
# Terminal 1 – start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2 – start the server
ros2 run lesson6_cmake move_turtlesim_server_exe

# Terminal 3 – run the client (triggers all four paths)
ros2 run lesson6_cmake move_turtlesim_client_exe
```
