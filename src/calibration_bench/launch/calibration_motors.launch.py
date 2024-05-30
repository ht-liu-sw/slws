from launch import LaunchDescription
from launch_ros.actions import Node, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

class motors_can_handle:
    pass
    
class parameters:
    
    pass
def generate_launch_description():
    ld = LaunchDescription()
    
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('path_to_included_launch.py')
    )
    
    set_can_up_cmd = ExecuteProcess(
        cmd = ['sudo ip link set can0 up type can bitrate 1000000'],
        output='screen'
    )
    ld.add_action(set_can_up_cmd)
    LaunchDescription([
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
    return ld
