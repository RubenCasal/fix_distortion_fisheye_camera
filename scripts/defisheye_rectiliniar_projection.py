#!/usr/bin/env python3

import numpy as np
import math
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2#!/usr/bin/env python3
from cv_bridge import CvBridge

class DefisheyeRectiliniarProjectionNode(Node):
    def __init__(self):
        super().__init__("defisheye_rectiliniar_projection_node")
        self.bridge = CvBridge()

        #Suscribe to any of the fisheye cameras topics
        self.subscription = self.create_subscription(
            Image,
            "/rs_t265/fisheye_left",
            self.image_callback,
            10
        )

        # Publisher for the processed image
        self.publisher = self.create_publisher(Image, "/rs_t265/defisheye_rectiliniar_projection_node", 10)

        self.get_logger().info("Defisheye node started")


    def rectify_image(self, input_image):
        """Rectifies fisheye distortion using a meshgrid transformation."""
        height, width = input_image.shape[:2]  # Correctly get image dimensions

        # Calculate variables for rectilinear projection
        midy = height // 2
        midx = width // 2
        maxmag = max(midy, midx)
        circum = int(2 * math.pi * maxmag)

        # Create a mesh grid of x and y coordinates
        x, y = np.meshgrid(np.arange(circum), np.arange(maxmag))

        # Calculate the corresponding theta and mag values
        theta = -1.0 * x / maxmag
        mag = maxmag - y

        mag = np.power(mag/maxmag,1.3) *maxmag

        # Compute the corresponding target coordinates
        targety = np.rint(midy + mag * np.cos(theta)).astype(int)
        targetx = np.rint(midx + mag * np.sin(theta)).astype(int)

        
        targety = np.clip(targety, 0, height - 1)
        targetx = np.clip(targetx, 0, width - 1)

        # Initialize output image
        output_image = np.zeros((maxmag, circum), dtype=np.uint8)

        # Assign pixel values from the input image
        output_image = input_image[targety, targetx]  

        return output_image  



    def image_callback(self, msg):
        "Callback fuction that receives a fisheye image and return a rectangular image"
        try:
            # Convert ROS2 image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            
            # Defish applied
            defish_image = self.rectify_image(cv_image)

            processed_msg = self.bridge.cv2_to_imgmsg(defish_image, encoding="mono8")
            processed_msg.header = msg.header 

            #Publish the processed image
            self.publisher.publish(processed_msg)
            self.get_logger().info("Published defished image")

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = DefisheyeRectiliniarProjectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()