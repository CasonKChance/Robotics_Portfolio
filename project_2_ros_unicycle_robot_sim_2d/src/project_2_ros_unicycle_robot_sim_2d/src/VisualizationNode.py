#!/usr/bin/env python3

import math
import threading

import matplotlib.patches as patches
import matplotlib.pyplot as plt

from project_2_ros_unicycle_robot_sim_2d.msg import RobotPose
from project_2_ros_unicycle_robot_sim_2d.srv import SendWorldData

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


class VisualizationNode(Node):

    def __init__(self):
        super().__init__('visualization_node')

        # Set up client
        self.world_client = self.create_client(
            SendWorldData,
            'send_world_data'
        )

        while not self.world_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Waiting for service from SimulationNode...'
            )

        # Set up /robot_pose subscriber
        self.robot_pose_subscription = self.create_subscription(
            RobotPose,
            'robot_pose',
            self.robot_pose_listener_callback,
            10)

        # World State variables
        self.max_x = None
        self.max_y = None
        self.goals = []
        self.obstacles = []
        self.robot_pose = None

        self.world_received = False
        self.world_drawn = False

        # Incremented whenever a new robot pose is received.
        self.robot_pose_version = 0
        self.last_drawn_pose_version = 0

        # Protects shared state between the ROS executor thread and
        # Matplotlib's GUI thread.
        self.state_lock = threading.Lock()

        # Variables for plot
        self.fig = None
        self.ax = None
        self.update_timer = None

        self.goal_artists = []
        self.obstacle_artists = []

        self.robot_body = None
        self.robot_heading = None
        self.robot_path = []

        # Send request
        self.request_world()

    def request_world(self):
        request = SendWorldData.Request()

        future = self.world_client.call_async(request)
        future.add_done_callback(self.world_response_callback)

    def world_response_callback(self, future):
        try:
            response = future.result()
        except Exception as exception:
            self.get_logger().error(
                f'Failed to receive world data: {exception}'
            )
            return

        with self.state_lock:
            self.max_x = response.max_x
            self.max_y = response.max_y
            self.goals = list(response.goal)
            self.obstacles = list(response.obstacles)
            self.robot_pose = response.robot_pose
            self.world_received = True

        self.get_logger().info(
            'Received world data from SimulationNode.'
        )

    def robot_pose_listener_callback(self, message):
        with self.state_lock:
            self.robot_pose = message
            self.robot_pose_version += 1

    def create_visualization(self):
        self.fig, self.ax = plt.subplots(
            figsize=(8, 8),
            dpi=120
        )

        self.ax.set_title(
            '2D Spatial Trajectory',
            fontweight='bold',
            loc='center'
        )

        self.ax.set_xlabel('X Position [m]')
        self.ax.set_ylabel('Y Position [m]')

        self.ax.set_aspect('equal', adjustable='box')

        self.ax.grid(
            True,
            linestyle='--',
            alpha=0.6
        )

        # Timer to update the plot when new robot pose data is received
        self.update_timer = self.fig.canvas.new_timer(
            interval=30,
            callbacks=[
                (self.update_visualization, [], {})
            ]
        )

        self.update_timer.start()

    def update_visualization(self):
        self.draw_world()
        self.update_robot()

        if self.fig is not None:
            self.fig.canvas.draw_idle()

    def draw_world(self):
        if self.world_drawn:
            return

        with self.state_lock:
            if not self.world_received:
                return

            max_x = self.max_x
            max_y = self.max_y
            goals = list(self.goals)
            obstacles = list(self.obstacles)
            robot_pose = self.robot_pose

        legend_handles = []

        world_boundary = patches.Rectangle(
            (0.0, 0.0),
            max_x,
            max_y,
            linewidth=2,
            edgecolor='black',
            facecolor='none'
        )

        self.ax.add_patch(world_boundary)

        for goal in goals:
            goal_circle = patches.Circle(
                (goal.center.x, goal.center.y),
                goal.radius,
                facecolor='gold',
                edgecolor='darkgoldenrod',
                alpha=0.6,
                linewidth=2
            )

            self.ax.add_patch(goal_circle)
            self.goal_artists.append(goal_circle)

        if goals:
            legend_handles.append(
                patches.Patch(
                    facecolor='gold',
                    edgecolor='darkgoldenrod',
                    alpha=0.6,
                    label='Goal Region'
                )
            )

        for obstacle in obstacles:
            obstacle_circle = patches.Circle(
                (obstacle.center.x, obstacle.center.y),
                obstacle.radius,
                facecolor='red',
                edgecolor='darkred',
                alpha=0.4,
                linewidth=1.5
            )

            self.ax.add_patch(obstacle_circle)
            self.obstacle_artists.append(obstacle_circle)

        if obstacles:
            legend_handles.append(
                patches.Patch(
                    facecolor='red',
                    edgecolor='darkred',
                    alpha=0.4,
                    label='Obstacle'
                )
            )

        if robot_pose is not None:
            robot_radius = 0.25
            heading_length = 0.5

            self.robot_body = patches.Circle(
                (robot_pose.x, robot_pose.y),
                robot_radius,
                facecolor='white',
                edgecolor='black',
                linewidth=2,
                zorder=5
            )

            self.ax.add_patch(self.robot_body)

            heading_x = (robot_pose.x + heading_length * math.cos(robot_pose.theta))
            heading_y = (robot_pose.y + heading_length * math.sin(robot_pose.theta))

            self.robot_heading, = self.ax.plot(
                [robot_pose.x, heading_x],
                [robot_pose.y, heading_y],
                linewidth=2,
                zorder=6
            )

            # Draw initial marker for first trajectory point
            self.ax.plot(
                robot_pose.x,
                robot_pose.y,
                marker='o',
                linestyle='None',
                color='blue',
                markersize=4,
                zorder=4
            )

            legend_handles.append(
                patches.Patch(
                    facecolor='white',
                    edgecolor='black',
                    linewidth=2,
                    label='Robot'
                )
            )

        self.ax.set_xlim([0.0, max_x])
        self.ax.set_ylim([0.0, max_y])

        self.ax.legend(
            handles=legend_handles,
            bbox_to_anchor=(1.0, 1.15),
            loc='upper right',
            frameon=True,
            fontsize=8
        )

        self.fig.tight_layout()

        self.world_drawn = True

        self.get_logger().info(
            'World visualization created.'
        )

    def update_robot(self):
        if not self.world_drawn:
            return

        with self.state_lock:
            if self.robot_pose is None:
                return

            if self.robot_pose_version == self.last_drawn_pose_version:
                return

            robot_pose = self.robot_pose

            self.last_drawn_pose_version = self.robot_pose_version

        heading_length = 0.5

        self.robot_body.center = (
            robot_pose.x,
            robot_pose.y
        )

        heading_x = (robot_pose.x + heading_length * math.cos(robot_pose.theta))
        heading_y = (robot_pose.y + heading_length * math.sin(robot_pose.theta))

        self.robot_heading.set_data([robot_pose.x, heading_x], [robot_pose.y, heading_y])

        # Leave a blue dot at the robot's new position.
        self.ax.plot(
            robot_pose.x,
            robot_pose.y,
            marker='o',
            linestyle='None',
            color='blue',
            markersize=4,
            zorder=4
        )

    def shutdown_visualization(self):
        if self.update_timer is not None:
            self.update_timer.stop()

        if self.fig is not None:
            plt.close(self.fig)


def main(args=None):

    rclpy.init(args=args)

    visualization_node = VisualizationNode()

    # Run ROS callbacks in a background thread so that the Matplotlib
    # GUI event loop can remain on the main thread.
    executor = SingleThreadedExecutor()
    executor.add_node(visualization_node)

    ros_thread = threading.Thread(
        target=executor.spin,
        daemon=True
    )

    ros_thread.start()

    try:
        # Matplotlib owns the main thread.
        visualization_node.create_visualization()
        plt.show()

    except KeyboardInterrupt:
        pass

    finally:
        executor.shutdown()

        visualization_node.shutdown_visualization()
        visualization_node.destroy_node()

        rclpy.shutdown()

        ros_thread.join()


if __name__ == '__main__':
    main()
