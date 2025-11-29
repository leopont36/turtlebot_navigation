from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ir_launch_dir = get_package_share_directory('ir_launch')
    ir_launch_file = os.path.join(ir_launch_dir, 'launch', 'assignment_1.launch.py')

    apriltag_pkg = get_package_share_directory('group18_apriltag_ros')
    apriltag_file = os.path.join(apriltag_pkg, 'launch', 'camera_36h11.launch.yml')

    assignment_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(ir_launch_file))
    
    apriltag_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(apriltag_file)
    )

    lifecycle_manager_node = Node(
        package='group18_mission_control',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen'
    )
    
    initial_pose_setter = Node(
        package='group18_mission_control',
        executable='initial_pose_setter',
        name='initial_pose_setter',
        output='screen'
    )

    tables_detection_node = Node(
        package='group18_mission_control',
        executable='tables_detection',
        name='tables_detection',
        output='screen',
    )

    navigate_client_node = Node(
        package='group18_mission_control',
        executable='navigate_to_pose_client',
        name='navigate_to_pose_client',
        output='screen',
        #parameters=[{'use_sim_time': True}] 
    )

    corridor_navigator_node = Node(
        package='group18_mission_control',
        executable='corridor_navigator',
        name='corridor_navigation_node',
        output='screen',
    )

    return LaunchDescription([
        assignment_launch,
        TimerAction(
            period=8.0,
            actions=[lifecycle_manager_node]
        ),
        TimerAction(
            period=5.0,
            actions=[initial_pose_setter]
        ),
        TimerAction(
            period=5.0,
            actions=[tables_detection_node]
        ),
        apriltag_launch,
        TimerAction(
            period=6.0,
            actions=[navigate_client_node]
        ),
        TimerAction(
            period=7.0,
            actions=[corridor_navigator_node]
        )
    ])  
