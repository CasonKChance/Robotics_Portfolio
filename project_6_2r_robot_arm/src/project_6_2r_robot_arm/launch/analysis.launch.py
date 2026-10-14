from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('project_6_2r_robot_arm')
    xacro_file = PathJoinSubstitution([package_share,
                                       'urdf', '2r_robot_arm.urdf.xacro'])
    robot_description = Command(['xacro', ' ', xacro_file])

    robot_state_publisher = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description}],
            )

    robot_arm_analyzer = Node(
        package='project_6_2r_robot_arm',
        executable='RobotArmAnalyzer',
        name='RobotArmAnalyzer',
        output='screen',
        emulate_tty=True
    )

    analysis_visualization = Node(
        package='project_6_2r_robot_arm',
        executable='AnalysisVisualization',
        name='AnalysisVisualization',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        robot_state_publisher,
        robot_arm_analyzer,
        analysis_visualization
    ])
