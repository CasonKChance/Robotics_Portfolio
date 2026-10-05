from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('project_4_differential_drive_robot')
    xacro_file = PathJoinSubstitution([package_share, 'urdf', 'differential_drive_robot.urdf.xacro'])
    robot_description = Command(['xacro', ' ', xacro_file])

    robot_state_publisher = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description}],
            )

    robot = Node(
                package='project_4_differential_drive_robot',
                executable='Robot',
                name='Robot',
                output='screen',
                emulate_tty=True
            )

    rviz_config = PathJoinSubstitution([
        package_share,
        'rviz',
        'differential_drive_robot.rviz'
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            rviz_config
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        robot,
        rviz
    ])