from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='calibration_bench',
            executable='calibration_bench',
            parameters=[{'calibration_bench_name': 'bench1'},
                        {'motor_xy_name': 'bench1_motor_xy'},
                        {'motor_xz_name': 'bench1_motor_xz'},
                        {'motor_xy_can_id': 0x0B},
                        {'motor_xz_can_id': 0x03},
                        {"can_interface_name": "can0"},
                        {"motor_xy_offset": 0.0},
                        {"motor_xz_offset": 0.0}],
            name="bench1"
        ),
    ])
