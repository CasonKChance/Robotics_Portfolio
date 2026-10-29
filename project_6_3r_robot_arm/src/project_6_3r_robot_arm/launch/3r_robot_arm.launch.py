from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='default',
        description='Operating mode for RobotArm'
    )

    goal_position_tolerance = LaunchConfiguration('goal_position_tolerance');
    goal_position_tolerance_arg = DeclareLaunchArgument(
        'goal_position_tolerance',
        default_value='0.01',
        description='End-effector goal positional tolerance in meters'
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

    robot_arm_controller = Node(
        package='project_6_3r_robot_arm',
        executable='RobotArmController',
        name='RobotArmController',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'IK'"])
        ),
        parameters=[{
            'goal_position_tolerance': ParameterValue(
                goal_position_tolerance,
                value_type=float
            )
        }]
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
        goal_position_tolerance_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        robot_arm,
        robot_arm_controller,
        rviz
    ])
