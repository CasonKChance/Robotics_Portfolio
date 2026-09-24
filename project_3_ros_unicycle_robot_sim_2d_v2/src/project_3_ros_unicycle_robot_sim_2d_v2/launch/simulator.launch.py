import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory('project_3_ros_unicycle_robot_sim_2d_v2')

    world_config = os.path.join(package_share_dir, 'config', 'world.yaml')
    robot_config = os.path.join(package_share_dir, 'config', 'robot.yaml')

    simulator_node = Node(
                        package='project_3_ros_unicycle_robot_sim_2d_v2',
                        executable='SimulatorNode',
                        name='Simulator',
                        parameters=[world_config, robot_config],
                        output='screen',
                        emulate_tty=True
                    )

    visualization_node = Node(
                            package='project_3_ros_unicycle_robot_sim_2d_v2',
                            executable='VisualizationNode',
                            name='Visualization',
                            output='screen',
                            emulate_tty=True
                        )

    robot_controller_node = Node(
                                package='project_3_ros_unicycle_robot_sim_2d_v2',
                                executable='RobotControllerNode',
                                name='RobotController',
                                parameters=[robot_config],
                                output='screen',
                                emulate_tty=True
                            )

    return LaunchDescription([
        simulator_node,
        visualization_node,
        robot_controller_node
    ])
