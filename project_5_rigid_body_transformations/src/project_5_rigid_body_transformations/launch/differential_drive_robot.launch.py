from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('project_5_rigid_body_transformations')
    xacro_file = PathJoinSubstitution([package_share,
                                       'urdf', 'differential_drive_robot.urdf.xacro'])
    robot_description = Command(['xacro', ' ', xacro_file])

    robot_state_publisher = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description}],
            )

    rviz_config = PathJoinSubstitution([
        package_share,
        'rviz',
        'differential_drive_robot.rviz'
    ])

    robot = Node(
                package='project_5_rigid_body_transformations',
                executable='Robot',
                name='Robot',
                output='screen',
                emulate_tty=True
            )

    robot_controller = Node(
                        package='project_5_rigid_body_transformations',
                        executable='RobotController',
                        name='RobotController',
                        output='screen',
                        emulate_tty=True
                       )

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

    landmark = Node(
        package='project_5_rigid_body_transformations',
        executable='Landmark',
        name='Landmark',
        output='screen',
        emulate_tty=True
    )

    landmark_sensor = Node(
        package='project_5_rigid_body_transformations',
        executable='LandmarkSensor',
        name='LandmarkSensor',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz,
        robot,
        robot_controller,
        landmark,
        landmark_sensor
    ])
