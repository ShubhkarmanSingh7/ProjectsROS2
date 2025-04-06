import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DemoPublisher(Node):
    def __init__(self):
        super().__init__('demo_publisher')
        self.publisher_ = self.create_publisher(String, '/demo_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = 'Hello! ROS2 is fun.'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = DemoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
