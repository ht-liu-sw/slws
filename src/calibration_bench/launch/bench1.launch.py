from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='calibration_bench',
            executable='calibration_bench',
            parameters=[{'calibration_bench_name': 'value'},
                        {'motor_xy_name': 'value'},
                        {'motor_xz_name': 'value'},
                        {'DeviceInd': 0},
                        {'CANChannel': 0},
                        {'motor_xy_frame_id': 0x2333},
                        {'motor_xz_frame_id': 0x4567}],
        ),
    ])
