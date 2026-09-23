import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import launch

def generate_launch_description():

    obstacle_arg = DeclareLaunchArgument(
        "obstacle", default_value="0.5"
    )
    degrees_arg = DeclareLaunchArgument(
        "degrees", default_value="-90"
    )

    final_approach_arg = DeclareLaunchArgument(
        "final_approach", default_value="false"
    )

    obstacle_f = LaunchConfiguration('obstacle')
    degrees_f = LaunchConfiguration('degrees')
    final_approach_f = LaunchConfiguration('final_approach')

    pre_approach_node = Node(
        package='attach_shelf',
        executable='pre_approach_v2_exe',
        output='screen',
        name='pre_approach_node',
        emulate_tty=True,
        parameters=[{
            'obstacle': obstacle_f,
            'degrees': degrees_f,
            'final_approach': final_approach_f,
            'use_sim_time': True
        }]
    )

    approach_service_node = Node(
        package='attach_shelf',
        executable='approach_service',
        output='screen',
        name='approach_service_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(
        'attach_shelf'), 'rviz', 'my_robot_v2.rviz')

    # This is to publish messages inside Launch files.
    message_info = launch.actions.LogInfo(
        msg=str(rviz_config_dir))

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir])

    

    # create and return launch description object
    return LaunchDescription(
        [
            obstacle_arg,
            degrees_arg,
            final_approach_arg,
            pre_approach_node,
            rviz_node,
            message_info,
            approach_service_node
        ]
    )