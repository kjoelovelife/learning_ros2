from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(package='lesson12_cmake', executable='lifecycle_example_exe',
                      name='managed_scan_node', namespace='', output='screen')
    ])