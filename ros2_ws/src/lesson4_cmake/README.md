# lesson4_cmake

C++ publisher/subscriber basics using `rclcpp`.

## Nodes

| Executable | Node class | Description |
|------------|-----------|-------------|
| `publisher_exe` | `Publisher` | Publishes a counter string on topic `sentence` every second |
| `subscriber_exe` | `Subscriber` | Subscribes to `sentence` and logs each message |

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/sentence` | `std_msgs/String` | Publisher → Subscriber |

## Usage

```bash
# Terminal 1 – start the publisher
ros2 run lesson4_cmake publisher_exe

# Terminal 2 – start the subscriber
ros2 run lesson4_cmake subscriber_exe

# Or launch both together
ros2 launch lesson4_cmake multiple.launch.py

# Turtlesim path exercise (requires turtlesim to be running)
ros2 launch lesson4_cmake turtlesim_path.launch.py
```

## Exercises

- `turtle_draw_circle` (launch): practice topic for publishing `geometry_msgs/Twist` to `/turtle1/cmd_vel` to draw a circle. Reference answer in `launch/tutorial_answer/`.
