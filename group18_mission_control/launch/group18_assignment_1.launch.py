from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ir_launch_dir = get_package_share_directory('ir_launch')
    ir_launch_file = os.path.join(ir_launch_dir, 'launch', 'assignment_1.launch.py')

    assignment_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(ir_launch_file))
    lifecycle_manager_node = Node(
        package='group18_mission_control',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen'
    )
    
    initial_pose_publisher = Node(
        package='group18_mission_control',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        output='screen'
    )   

    return LaunchDescription([
        assignment_launch,
        TimerAction(
            period=5.0,
            actions=[lifecycle_manager_node]
        ),
        TimerAction(
            period=6.0,
            actions=[initial_pose_publisher]
        )
    ])  
