from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_gz_gui = LaunchConfiguration('launch_gz_gui')
    launch_gz_gui_arg = DeclareLaunchArgument(
        'launch_gz_gui',
        default_value=False,
        description='Boolean for launching the Gazebo GUI'
    )

    package_share = FindPackageShare('project_7_treasure_hunting_robot')
    gz_package_share = FindPackageShare('ros_gz_sim')

    # Bridge
    bridge_config = PathJoinSubstitution([
        package_share,
        'config',
        'bridge.yaml'
    ])

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # Camera bridge
    gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    # Robot Description
    differential_drive_model_xacro_file = PathJoinSubstitution([
        package_share,
        'models', 
        'differential_drive_robot.urdf.xacro'
    ])

    differential_drive_robot_description = Command(['xacro', ' ', differential_drive_model_xacro_file])

    # Gazebo world
    treasure_hunting_world_file = PathJoinSubstitution([
        package_share,
        'models',
        'treasure_hunting_world.sdf'
    ])

    # Gazebo with GUI
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                gz_package_share,
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': ['-r ', treasure_hunting_world_file]
        }.items(),
        condition=IfCondition(launch_gz_gui)
    )

    # Gazebo server only
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                gz_package_share,
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': ['-s -r ', treasure_hunting_world_file]
        }.items(),
        condition=UnlessCondition(launch_gz_gui)
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
            '-z', '0',
            '-R', '0',
            '-P', '0',
            '-Y', '1.57'
        ]
    )

    # RViz
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
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    # Robot Controller
    robot_controller = Node(
        package='project_7_treasure_hunting_robot',
        executable='RobotController',
        name='RobotController',
        output='screen',
        emulate_tty=True
    )

    # Velocimeter
    velocimeter = Node(
        package='project_7_treasure_hunting_robot',
        executable='Velocimeter',
        name='Velocimeter',
        output='screen',
        emulate_tty=True,
        prefix='xterm -title "Velocimeter" -geometry 48x10 -e',
        parameters=[{'use_sim_time': True}]
    )

    # Image Decoder
    image_decoder = Node(
        package='project_7_treasure_hunting_robot',
        executable='ImageDecoder',
        name='ImageDecoder',
        output='screen',
        emulate_tty=True
    )

    # Coordinates Processor
    coordinates_processor = Node(
        package='project_7_treasure_hunting_robot',
        executable='CoordinatesProcessor',
        name='CoordinatesProcessor',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        launch_gz_gui_arg,
        gz_bridge,
        gz_image_bridge,
        gazebo_gui,
        gazebo_server,
        robot_state_publisher,
        spawn_robot,
        rviz,
        robot_controller,
        velocimeter,
        image_decoder,
        coordinates_processor
    ])
