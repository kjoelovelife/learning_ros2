# lesson10_python

Demonstrates subscribing to `sensor_msgs/LaserScan` data, filtering it by angle range, and republishing — along with RViz visualization.

## Dependencies

Requires `lesson_interfaces` to be built first.

## Nodes

| Executable | Node class | Description |
|------------|-----------|-------------|
| `retrieve_laser_scan_exe` | `RetrieveLaserScan` | Subscribes to `/scan`, trims the scan to a configurable angle range, and republishes on `/custom_scan` |

## Parameters

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `scan_angle_range` | `double` | `270.0` | 0–270 deg | Maximum angle range to retain from the incoming scan |

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/scan` | `sensor_msgs/LaserScan` | Sensor → RetrieveLaserScan |
| `/custom_scan` | `sensor_msgs/LaserScan` | RetrieveLaserScan → RViz / consumers |

## Usage

```bash
# Launch with default 150° scan angle (config/retrieve_laser_scan.yaml)
ros2 launch lesson10_python retrieve_laser_scan.launch.py

# Launch turtlesim with custom background color (config/turtlesim.yaml)
ros2 launch lesson10_python turtlesim.launch.py
```

RViz configuration is provided in `rviz/laser_scan.rviz`.

### Change scan angle at runtime

```bash
ros2 param set /retrieve_laser_scan_node scan_angle_range 90.0
```
