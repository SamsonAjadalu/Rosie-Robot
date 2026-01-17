import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class TargetPointPublisher(Node):
    def __init__(self):
        super().__init__('target_point_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/target_point', 10)
        self.publish_messages()

    def publish_messages(self):
        points = [



            # Ready to pick
            [-0.302, -0.403, 0.194,
             0.982, -0.11, -0.00, 0.15],


        ]

        # Publish each set of points
        for idx, point in enumerate(points):
            msg = Float64MultiArray()
            msg.data = point
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published message {idx + 1}: {msg.data}')
            rclpy.spin_once(self, timeout_sec=6.5)  # Add a small delay between messages

        self.destroy_node()  # Clean up the node properly after publishing

def main(args=None):
    rclpy.init(args=args)
    node = TargetPointPublisher()
    rclpy.shutdown()  # Shutdown rclpy after all messages are published

if __name__ == '__main__':
    main()
