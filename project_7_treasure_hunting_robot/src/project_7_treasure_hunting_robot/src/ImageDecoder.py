#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from project_7_treasure_hunting_robot_interfaces.msg import HintCoordinates
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageDecoder(Node):
    """
    @brief Processes camera images to identify colored hints and decode their coordinates.
    """

    def __init__(self):
        """
        @brief Constructs the image decoder and initializes ROS interfaces and image processing.
        """

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
            HintCoordinates,
            'hint_coordinates',
            10
        )

    def image_topic_callback(self, msg: Image):
        """
        @brief Converts a received ROS image into an OpenCV image for processing.
        @param msg ROS image message received from the camera
        """

        self.image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

    def decode_image(self):
        """
        @brief Identifies colored hints in the current image and publishes their associated coordinates.
        """

        if self.image is None:
            return

        dominant_color, dominance = self.pull_dominant_color()

        if dominant_color is not None and dominance >= 0.95:
            is_blue_channel_present = dominant_color[0] >= 253
            is_green_channel_present = dominant_color[1] >= 253
            is_red_channel_present = dominant_color[2] >= 253

            if is_blue_channel_present and is_green_channel_present and is_red_channel_present:
                # White is the floor's color and gives us no information
                return
            elif is_blue_channel_present and is_green_channel_present:
                # Cyan circle is hint #1 and gives us the location of the blue circle at (3.0, -2.5)
                next_coordinates = (3.0, -2.5)
                current_coordinates = (-0.5, -1.0)
                self.publish_coordinates(*next_coordinates, *current_coordinates)
                return
            elif is_blue_channel_present and is_red_channel_present:
                # Purple circle is hint #3 and gives us the location of the yellow circle at (-3.5, 3.0)
                next_coordinates = (-3.5, 3.0)
                current_coordinates = (-3.0, -4.0)
                self.publish_coordinates(*next_coordinates, *current_coordinates)
                return
            elif is_green_channel_present and is_red_channel_present:
                # Yellow circle is hint #4 and gives us the location of the red circle at (4.0, 3.5)
                next_coordinates = (4.0, 3.5)
                current_coordinates = (-3.5, 3.0)
                self.publish_coordinates(*next_coordinates, *current_coordinates)
                return
            elif is_blue_channel_present:
                # Blue circle is hint #2 and gives us the location of the purple circle at (-3.0, -4.0)
                next_coordinates = (-3.0, -4.0)
                current_coordinates = (3.0, -2.5)
                self.publish_coordinates(*next_coordinates, *current_coordinates)
                return
            elif is_green_channel_present:
                # Green circle is hint #0 and gives us the location of the cyan circle at (-0.5, -1.0)
                next_coordinates = (-0.5, -1.0)
                current_coordinates = (0.0, 0.0)
                self.publish_coordinates(*next_coordinates, *current_coordinates)
                return
            elif is_red_channel_present:
                # The red circle contains the treasure
                self.get_logger().info(
                    f"Treasure found!",
                    once=True
                )
                return
    

    def pull_dominant_color(self):
        """
        @brief Determines the dominant BGR color in the current image using K-means clustering.
        @return Tuple containing the dominant BGR color and its proportion of the image, or None if no image is available
        """

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

    def publish_coordinates(self, next_x, next_y, current_x, current_y):
        """
        @brief Publishes the current hint coordinates and the location of the next hint.
        @param next_x X coordinate of the next hint
        @param next_y Y coordinate of the next hint
        @param current_x X coordinate of the current hint (the robot's current approximate x position)
        @param current_y Y coordinate of the current hint (the robot's current approximate y position)
        """

        msg = HintCoordinates()

        msg.next_hint_x = float(next_x)
        msg.next_hint_y = float(next_y)
        msg.next_hint_z = float(0)
        msg.current_hint_x = float(current_x)
        msg.current_hint_y = float(current_y)
        msg.current_hint_z = float(0)

        self.get_logger().debug(
            f"Publishing coordinates (x, y): {next_x, next_y}"
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
