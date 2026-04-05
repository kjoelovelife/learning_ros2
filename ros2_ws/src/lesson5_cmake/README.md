# lesson5_cmake

C++ publisher/subscriber using a custom message type (`lesson_interfaces/Lunch`).

## Dependencies

Requires `lesson_interfaces` to be built first.

## Nodes

| Executable | Node class | Description |
|------------|-----------|-------------|
| `publisher_exe` | `LunchInfoPublisher` | Publishes `Lunch` message (rice, bowl color, vegetables, meats) on `lunch_info` every 0.5 s |
| `subscriber_exe` | `LunchInfoSubscriber` | Subscribes to `lunch_info` and logs meal details |

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/lunch_info` | `lesson_interfaces/Lunch` | Publisher → Subscriber |

## Usage

```bash
ros2 launch lesson5_cmake lunch_info.launch.py
```
