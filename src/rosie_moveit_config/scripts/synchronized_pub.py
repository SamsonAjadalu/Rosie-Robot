#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, CameraInfo

class DepthCameraSynchronizer(Node):
    def __init__(self):
        super().__init__("depth_camera_synchronizer")

        # Subscribers for Isaac topics
        self.depth_sub = Subscriber(self, Image, "/isaac/depth")  # Replace with your Isaac depth topic
        self.camera_info_sub = Subscriber(self, CameraInfo, "/isaac/camera_info")  # Replace with your Isaac camera info topic

        # Publishers for synchronized standard topics
        self.synced_depth_pub = self.create_publisher(Image, "/depth", 10)
        self.synced_camera_info_pub = self.create_publisher(CameraInfo, "/camera_info", 10)

        # Synchronizer: ApproximateTimeSynchronizer
        self.sync = ApproximateTimeSynchronizer(
            [self.depth_sub, self.camera_info_sub],
            queue_size=5,
            slop=1.0,  # Time difference tolerance in seconds
        )
        self.sync.registerCallback(self.callback)

        self.get_logger().info("Depth camera synchronizer initialized")

    def callback(self, depth_msg, camera_info_msg):
        # Republish synchronized messages
        self.synced_depth_pub.publish(depth_msg)
        self.synced_camera_info_pub.publish(camera_info_msg)
        self.get_logger().info("Republished synchronized depth and camera info messages")

def main(args=None):
    rclpy.init(args=args)
    node = DepthCameraSynchronizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
