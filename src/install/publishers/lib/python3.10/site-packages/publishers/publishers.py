import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MapPublisher():
    def __init__(self):
        super().__init__("map_node")
        self.publisher = self.create_publisher(String, "map", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.map = {}

    def timer_callback(self):
        msg = String()
        msg.data = f"Map: {self.map}"

class StartPublisher():
    def __init__(self):
        super().__init__("start_node")
        self.publisher = self.create_publisher(String, "start", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.start = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"Start: {self.start}"

class GoalPublisher():
    def __init__(self):
        super().__init__("goal_node")
        self.publisher = self.create_publisher(String, "goal", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.goal = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"Goal: {self.goal}"

def main():
    rclpy.init()

    # create nodes
    mapNode = MapPublisher()
    startNode = StartPublisher()
    goalNode = GoalPublisher()

    # use nodes
    rclpy.spin(mapNode)
    rclpy.spin(startNode)
    rclpy.spin(goalNode)

    # destroy nodes
    mapNode.destroy_node()
    startNode.destroy_node()
    goalNode.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()
