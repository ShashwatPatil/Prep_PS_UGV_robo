import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_file_name = 'urdf/bambot_v2.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('gazebo_spawn_pkg'),
        'urdf',
        urdf_file_name
    )

    world_file = os.path.join(
        get_package_share_directory('gazebo_spawn_pkg'),
        'worlds',
        'explore.world'   # for task 2 'manipulation.world' 
    )


    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='xcb'),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'bambot',
                '-topic', '/robot_description',
                '-z', '0.1',
            ],
            output='screen'
        ),

        # Joint state broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),
        # Head Pitch controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['head_controller'],
            output='screen'
        ),
        # Right arm controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['right_arm_controller'],
            output='screen'
        ),

        # Left arm controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['left_arm_controller'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['left_wheel_controller'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['right_wheel_controller'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['back_wheel_controller'],
            output='screen'
        ),

        
    ])
