import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import time

class FrameBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_callback)
        self.angle = 0.0

    def broadcast_callback(self):
        t = TransformStamped()

        # 父坐标系 world，子坐标系 base_link
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        # 简单让机器人绕圆圈运动
        t.transform.translation.x = math.cos(self.angle)
        t.transform.translation.y = math.sin(self.angle)
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(t)
        self.get_logger().info(f'Broadcasting base_link at x={t.transform.translation.x:.2f}, y={t.transform.translation.y:.2f}')

        self.angle += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = FrameBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

