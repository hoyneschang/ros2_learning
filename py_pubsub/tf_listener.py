import rclpy
from rclpy.node import Node
import tf2_ros

class FrameListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'world',
                'base_link',
                now)
            self.get_logger().info(
                f"base_link position: x={trans.transform.translation.x:.2f}, y={trans.transform.translation.y:.2f}")
        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = FrameListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
