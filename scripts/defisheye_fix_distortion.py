#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class Defisheye:
    """Defisheye distortion correction algorithm."""
    def __init__(self, image, fov=163, pfov=120, dtype="equalarea", format="fullframe"):
        self.fov = fov  # Fisheye FoV (T265 = 163°)
        self.pfov = pfov  # Perspective FoV
        self.dtype = dtype  # Projection type (equalarea, linear, stereographic)
        self.format = format  # fullframe or circular

        self.image = image
        self.height, self.width = image.shape[:2]
        self.xcenter = self.width // 2
        self.ycenter = self.height // 2

    def _map(self, i, j, ofocinv, dim):
        """Computes new pixel positions based on fisheye mapping."""
        xd = i - self.xcenter
        yd = j - self.ycenter
        rd = np.hypot(xd, yd)
        phiang = np.arctan(ofocinv * rd)

        if self.dtype == "linear":
            ifoc = dim * 180 / (self.fov * np.pi)
            rr = ifoc * phiang
        elif self.dtype == "equalarea":
            ifoc = dim / (2.0 * np.sin(self.fov * np.pi / 720))
            rr = ifoc * np.sin(phiang / 2)
        elif self.dtype == "stereographic":
            ifoc = dim / (2.0 * np.tan(self.fov * np.pi / 720))
            rr = ifoc * np.tan(phiang / 2)

        rdmask = rd != 0
        xs = xd.astype(np.float32).copy()
        ys = yd.astype(np.float32).copy()

        xs[rdmask] = (rr[rdmask] / rd[rdmask]) * xd[rdmask] + self.xcenter
        ys[rdmask] = (rr[rdmask] / rd[rdmask]) * yd[rdmask] + self.ycenter
        xs[~rdmask] = 0
        ys[~rdmask] = 0

        return xs, ys

    def convert(self):
        """Applies fisheye correction."""
        dim = min(self.width, self.height) if self.format == "fullframe" else np.sqrt(self.width**2 + self.height**2)
        ofoc = dim / (2 * np.tan(self.pfov * np.pi / 360))
        ofocinv = 1.0 / ofoc

        i, j = np.meshgrid(np.arange(self.width), np.arange(self.height))
        xs, ys = self._map(i, j, ofocinv, dim)

        return cv2.remap(self.image, xs, ys, cv2.INTER_LINEAR)


class FisheyeCorrectionNode(Node):
    """ROS2 node that applies fisheye distortion correction."""
    def __init__(self):
        super().__init__("fisheye_correction_node")
        self.bridge = CvBridge()

        # Subscribe to T265 Fisheye Left Camera
        self.subscription = self.create_subscription(
            Image, "rs_t265/fisheye_left", 
            self.image_callback,
            10
        )

        # Publisher for corrected images
        self.publisher = self.create_publisher(Image, "rs_t265/undistorted", 10)

        self.get_logger().info("Fisheye correction node started.")

    def image_callback(self, msg):
        """Processes fisheye images and publishes corrected versions."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Apply Defisheye correction
            corrected_image = Defisheye(cv_image, fov=163, pfov=120, dtype="equalarea").convert()

            # Convert back to ROS2 Image message
            corrected_msg = self.bridge.cv2_to_imgmsg(corrected_image, encoding="bgr8")
            corrected_msg.header = msg.header

            # Publish the undistorted image
            self.publisher.publish(corrected_msg)
            self.get_logger().info("Published undistorted image.")

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = FisheyeCorrectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
