# lesson5_python

Python publisher/subscriber using a custom message type (`lesson_interfaces/Lunch`).

## Dependencies

Requires `lesson_interfaces` to be built first.

## Nodes

| Executable | Node class | Description |
|------------|-----------|-------------|
| `publisher` | `LunchInfoPublisher` | Publishes `Lunch` message (2 bowls of rice, purple bowl color, meats, vegetables) on `lunch_info` every 0.5 s |
| `subscriber` | `LunchInfoSubscriber` | Subscribes to `lunch_info` and logs meal details |

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/lunch_info` | `lesson_interfaces/Lunch` | Publisher → Subscriber |

## Usage

```bash
ros2 launch lesson5_python lunch_info.launch.py
```

Additional launch files:
- `teleop_autoridetb3_turtlesim.launch.py` — launches TurtleBot3 teleop with turtlesim
- `tutorial_answer/lunch_info_answer.launch.py` — reference answer launch
