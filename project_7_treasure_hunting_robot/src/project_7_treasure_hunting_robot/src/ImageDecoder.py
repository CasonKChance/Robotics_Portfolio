#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageDecoder(Node):
    def __init__(self):
        super().__init__('image_decoder')

        self.bridge = CvBridge()
        self.image = None

        self.subscription = self.create_subscription(
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

    def image_topic_callback(self, msg: Image):
        self.image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

    def decode_image(self):
        if self.image is None:
            return

        dominant_color, dominance = self.pull_dominant_color()

        if dominant_color is not None:
            self.get_logger().info(
                f"Dominant color: {dominant_color}, "
                f"Coverage: {dominance:.1%}"
            )

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
