from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='default',
        description='Operating mode for RobotArm'
    )

    package_share = FindPackageShare('project_6_3r_robot_arm')
    xacro_file = PathJoinSubstitution([package_share,
                                       'urdf', '3r_robot_arm.urdf.xacro'])
    robot_description = Command(['xacro', ' ', xacro_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", mode, "' != 'IK'"])
        )
    )

    robot_arm = Node(
        package='project_6_3r_robot_arm',
        executable='RobotArm',
        name='RobotArm',
        output='screen',
        emulate_tty=True,
        parameters=[{'mode': mode}]
    )

    rviz_config = PathJoinSubstitution([
        package_share,
        'rviz',
        '3r_robot_arm.rviz'
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
        mode_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        robot_arm,
        rviz
    ])
