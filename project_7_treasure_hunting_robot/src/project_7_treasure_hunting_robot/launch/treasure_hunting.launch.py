from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share_dir = FindPackageShare('project_7_treasure_hunting_robot')
    gz_package_share_dir = FindPackageShare('ros_gz_sim')

    # Robot Description
    differential_drive_model_xacro_file = PathJoinSubstitution([
        package_share_dir,
        'models', 
        'differential_drive_robot.urdf.xacro'
    ])

    differential_drive_robot_description = Command(['xacro', ' ', differential_drive_model_xacro_file])

    # Gazebo world
    treasure_hunting_world_file = PathJoinSubstitution([
        package_share_dir,
        'models',
        'treasure_hunting_world.sdf'
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                gz_package_share_dir,
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': ['-r ', treasure_hunting_world_file]
        }.items()
    )

    # Publish robot transforms
    robot_state_publisher = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': differential_drive_robot_description,
                    'use_sim_time': True
                }],
            )

    # Spawn robot into Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-world', 'treasure_hunting_world',
            '-topic', 'robot_description',
            '-name', 'differential_drive_robot',
            '-x', '0',
            '-y', '0',
            '-z', '0'
        ]
    )

    # RViz
    rviz_config = PathJoinSubstitution([
        package_share_dir,
        'rviz',
        'differential_drive_robot.rviz'
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        rviz,
    ])
