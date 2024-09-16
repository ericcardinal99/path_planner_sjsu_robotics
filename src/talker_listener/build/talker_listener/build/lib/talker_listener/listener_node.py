import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super.__init__("listener_node")
        self.subscriptions = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )
    
    def listener_callback(self):
        self.get_logger().info("Received " + self.msg.data)

def main():
    rclpy.init()

    listenerNode = ListenerNode()

    rclpy.spin(listenerNode)

    listenerNode.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()