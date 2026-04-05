# lesson8_python

A multi-node turtlesim game demonstrating publishers, subscribers, services, and P-controller motion. Three cooperating nodes work together to spawn, chase, and catch turtles.

## Dependencies

Requires `lesson_interfaces` to be built first.

## Nodes

| Executable | Node class | Description |
|------------|-----------|-------------|
| `turtle_spawner_exe` | `TurtleSpawner` | Spawns a new random turtle every 2 s via `turtlesim/Spawn`; publishes the alive turtle list; provides the `catch_turtle` service |
| `catch_the_turtle_exe` | `CatchTheTurtle` | Drives `turtle1` toward the nearest alive turtle using a P-controller; calls `catch_turtle` when within 0.5 units |
| `follow_the_turtle_exe` | `FollowTheTurtle` | Spawns `turtle2` and drives it toward `turtle1` using a P-controller |

## Node interaction diagram

```
turtlesim/Spawn  ←──  TurtleSpawner  ──→  alive_turtles (TurtleArray)
                             ↑                       ↓
                    catch_turtle srv          CatchTheTurtle
                             ↑                  /turtle1/cmd_vel
                    TurtleSpawner         /turtle1/pose
```

## Topics

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `alive_turtles` | `lesson_interfaces/TurtleArray` | TurtleSpawner | CatchTheTurtle |
| `/turtle1/pose` | `turtlesim/Pose` | turtlesim | CatchTheTurtle, FollowTheTurtle |
| `/turtle2/pose` | `turtlesim/Pose` | turtlesim | FollowTheTurtle |
| `/turtle1/cmd_vel` | `geometry_msgs/Twist` | CatchTheTurtle | turtlesim |
| `/turtle2/cmd_vel` | `geometry_msgs/Twist` | FollowTheTurtle | turtlesim |

## Services

| Service | Type | Server | Caller |
|---------|------|--------|--------|
| `/spawn` | `turtlesim/Spawn` | turtlesim | TurtleSpawner, FollowTheTurtle |
| `/kill` | `turtlesim/Kill` | turtlesim | TurtleSpawner |
| `/catch_turtle` | `lesson_interfaces/CatchTurtle` | TurtleSpawner | CatchTheTurtle |

## Usage

```bash
# Terminal 1 – start turtlesim
ros2 run turtlesim turtlesim_node

# Launch spawner + catcher together
ros2 launch lesson8_python catch_the_turtle.launch.py

# Or launch individual nodes
ros2 launch lesson8_python spawn_the_turtle.launch.py
ros2 launch lesson8_python follow_the_turtle.launch.py
```
