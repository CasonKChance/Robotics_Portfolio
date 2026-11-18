#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageDecoder(Node):
    def __init__(self):
        super().__init__('image_decoder')

        self.bridge = CvBridge()
        self.image = None

        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_topic_callback,
            10
        )

        # Process the most recent image at 5 Hz
        self.processing_timer = self.create_timer(
            0.2,
            self.decode_image
        )

        self.coordinates_publisher = self.create_publisher(
            Point,
            'hint_coordinates',
            10
        )

    def image_topic_callback(self, msg: Image):
        self.image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

    def decode_image(self):
        if self.image is None:
            return

        dominant_color, dominance = self.pull_dominant_color()

        if dominant_color is not None and dominance >= 0.75:
            is_blue_channel_present = dominant_color[0] >= 253
            is_green_channel_present = dominant_color[1] >= 253
            is_red_channel_present = dominant_color[2] >= 253

            if is_blue_channel_present and is_green_channel_present and is_red_channel_present:
                # White is the floor's color and gives us no information
                return
            elif is_blue_channel_present and is_green_channel_present:
                # Cyan circle is hint #1 and gives us the location of the blue circle at (3.0, -2.5)
                coordinates = (3.0, -2.5)
                self.publish_coordinates(*coordinates)
                return
            elif is_blue_channel_present and is_red_channel_present:
                # Purple circle is hint #3 and gives us the location of the yellow circle at (-3.5, 3.0)
                coordinates = (-3.5, 3.0)
                self.publish_coordinates(*coordinates)
                return
            elif is_green_channel_present and is_red_channel_present:
                # Yellow circle is hint #4 and gives us the location of the red circle at (4.0, 3.5)
                coordinates = (4.0, 3.5)
                self.publish_coordinates(*coordinates)
                return
            elif is_blue_channel_present:
                # Blue circle is hint #2 and gives us the location of the purple circle at (-3.0, -4.0)
                coordinates = (-3.0, -4.0)
                self.publish_coordinates(*coordinates)
                return
            elif is_green_channel_present:
                # Green circle is hint #0 and gives us the location of the cyan circle at (-0.5, -1.0)
                coordinates = (-0.5, -1.0)
                self.publish_coordinates(*coordinates)
                return
            elif is_red_channel_present:
                # The red circle contains the treasure
                self.get_logger().info(
                    f"Treasure found!",
                    once=True
                )
                return
    

    def pull_dominant_color(self):
        if self.image is None:
            return None

        # Resize to reduce computational cost
        image = cv2.resize(self.image, (100, 100))

        # Convert image into a list of BGR pixels
        pixels = image.reshape((-1, 3)).astype(np.float32)

        # Number of color clusters
        k = 5

        # K-means termination criteria
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            100,
            0.2
        )

        # Cluster pixels by color
        _, labels, centers = cv2.kmeans(
            pixels,
            k,
            None,
            criteria,
            10,
            cv2.KMEANS_PP_CENTERS
        )

        # Count pixels belonging to each cluster
        counts = np.bincount(labels.flatten(), minlength=k)

        # Find the largest cluster
        dominant_cluster = np.argmax(counts)

        # Return dominant color as a BGR tuple
        dominant_color = centers[dominant_cluster]

        # Percentage of image dominated by color
        dominance = counts[dominant_cluster] / len(pixels)

        return (
            tuple(int(channel) for channel in dominant_color),
            float(dominance)
        )

    def publish_coordinates(self, x, y):
        msg = Point()

        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(0)

        self.get_logger().debug(
            f"Publishing coordinates (x, y): {x, y}"
        )

        self.coordinates_publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)

    image_decoder_node = ImageDecoder()

    try:
        rclpy.spin(image_decoder_node)
    except KeyboardInterrupt:
        pass
    finally:
        image_decoder_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
