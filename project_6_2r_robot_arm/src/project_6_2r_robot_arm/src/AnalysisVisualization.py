#!/usr/bin/env python3

from geometry_msgs.msg import Point

import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class TaskSpacePlotter(Node):

    def __init__(self):
        super().__init__('task_space_plotter')

        qos = QoSProfile(
            depth=80000,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.subscription = self.create_subscription(
            Point,
            '/task_space',
            self.task_space_callback,
            qos
        )

        self.x_values = []
        self.z_values = []
        self.dirty = False

        # Configure graph.
        plt.ion()

        self.fig, self.ax = plt.subplots(figsize=(8, 8))

        self.ax.set_title('Task Space', y=1.15)

        # Fixed axis limits.
        self.ax.set_xlim(-1.0, 1.0)
        self.ax.set_ylim(-1.0, 1.0)

        # Equal scaling on both axes.
        self.ax.set_aspect('equal', adjustable='box')

        # Move axes to the origin.
        self.ax.spines['left'].set_position('zero')
        self.ax.spines['bottom'].set_position('zero')

        # Hide outer box spines.
        self.ax.spines['right'].set_color('none')
        self.ax.spines['top'].set_color('none')

        # Tick locations.
        self.ax.set_xticks([-1.0, 1.0])
        self.ax.set_yticks([-1.0, 1.0])

        self.ax.xaxis.set_ticks_position('bottom')
        self.ax.yaxis.set_ticks_position('left')

        # Remove default axis labels.
        self.ax.set_xlabel('')
        self.ax.set_ylabel('')

        # Put axis labels at the positive ends.
        self.ax.text(
            1.06, 0.0, 'x',
            transform=self.ax.transData,
            ha='left',
            va='center',
            fontsize=12
        )

        self.ax.text(
            0.0, 1.08, 'z',
            transform=self.ax.transData,
            ha='center',
            va='bottom',
            fontsize=12
        )

        self.ax.grid(True, alpha=0.25)

        # Create scatter plot with black dots.
        self.scatter = self.ax.scatter(
            [],
            [],
            c='black',
            s=10,
            marker='o'
        )

        self.fig.show()

        self.fig.subplots_adjust(top=0.78)

    def task_space_callback(self, msg):
        self.x_values.append(msg.x)
        self.z_values.append(msg.z)
        self.dirty = True

    def update_plot(self):
        if not self.dirty:
            return

        self.scatter.set_offsets(
            list(zip(self.x_values, self.z_values))
        )

        self.fig.canvas.draw_idle()
        self.dirty = False


def main(args=None):
    rclpy.init(args=args)
    node = TaskSpacePlotter()

    try:
        while rclpy.ok() and plt.fignum_exists(node.fig.number):
            # Process incoming ROS messages.
            for _ in range(1000):
                rclpy.spin_once(node, timeout_sec=0.001)

            # Update visualization.
            node.update_plot()
            plt.pause(0.05)

    except KeyboardInterrupt:
        pass

    finally:
        plt.close('all')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
