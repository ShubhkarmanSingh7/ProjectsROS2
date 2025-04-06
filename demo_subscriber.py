import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DemoSubscriber(Node):
    def __init__(self):
        super().__init__('demo_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/demo_topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = DemoSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
