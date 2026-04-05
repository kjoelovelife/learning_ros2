# lesson4_python

Python publisher/subscriber basics using `rclpy`.

## Nodes

| Executable | Node class | Description |
|------------|-----------|-------------|
| `publisher` | `Publisher` | Publishes a fixed string on topic `sentence` every second |
| `subscriber` | `Subscriber` | Subscribes to `sentence` and logs each message |
| `turtle_draw_circle_answer` | `TurtleDrawCircle` | Reference answer: publishes `Twist` to draw a circle in turtlesim |

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/sentence` | `std_msgs/String` | Publisher → Subscriber |
| `/turtle1/cmd_vel` | `geometry_msgs/Twist` | TurtleDrawCircle → turtlesim |

## Usage

```bash
# Terminal 1 – start the publisher
ros2 run lesson4_python publisher

# Terminal 2 – start the subscriber
ros2 run lesson4_python subscriber

# Or launch both
ros2 launch lesson4_python multiple.launch.py

# Turtlesim path exercise (requires turtlesim)
ros2 launch lesson4_python turtlesim_path.launch.py
```

## Exercises

- `turtle_draw_circle.py`: incomplete skeleton for publishing `geometry_msgs/Twist` to drive turtlesim in a circle. Fill in the `create_publisher` and `create_timer` calls, and implement `set_speed()`. Reference answer in `tutorial_answer/`.
- YAML parameter config in `config/parameter.yaml`.
