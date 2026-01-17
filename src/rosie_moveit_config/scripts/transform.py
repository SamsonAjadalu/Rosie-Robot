from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import rclpy

class TransformHandler(Node):
    def __init__(self):
        super().__init__('transform_handler')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def lookup_transform(self, parent_frame, child_frame):
        try:
            # Query the transform
            transform = self.tf_buffer.lookup_transform(
                parent_frame, child_frame, rclpy.time.Time()
            )
            self.get_logger().info(f"Transform from {parent_frame} to {child_frame}: {transform}")
            return transform
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"Error: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = TransformHandler()

    parent_frame = "base_link"
    child_frame = "right_gripper_base_link"


    try:
        while rclpy.ok():
            transform = node.lookup_transform(parent_frame, child_frame)
            if transform:
                print(f"Transform: {transform}")
            else:
                print("Transform not available, retrying...")
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
