import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MapPublisher(Node):
    def __init__(self):
        super().__init__("map_node")
        self.publisher = self.create_publisher(String, "map", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.map = [[1,1,1],[1,0,1],[1,1,1]]

    def timer_callback(self):
        msg = String()
        msg.data = f"Map: {self.map}"
        self.get_logger().info(f"{msg.data}")

class StartPublisher(Node):
    def __init__(self):
        super().__init__("start_node")
        self.publisher = self.create_publisher(String, "start", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.start = (0,0)

    def timer_callback(self):
        msg = String()
        msg.data = f"Start: {self.start}"
        self.get_logger().info(f"{msg.data}")

class GoalPublisher(Node):
    def __init__(self):
        super().__init__("goal_node")
        self.publisher = self.create_publisher(String, "goal", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.goal = (2,2)

    def timer_callback(self):
        msg = String()
        msg.data = f"Goal: {self.goal}"
        self.get_logger().info(f"{msg.data}")

class PathPublisher(Node):
    def __init__(self, map, start, goal):
        super().__init__("path_node")
        self.publisher = self.create_publisher(String, "goal", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.map = map
        self.start = start
        self.goal = goal
        self.obstacle_identifier = 0

    def get_path(self):
        distances = {}
        previous = {}
        unvisited = []

        for i in range(len(self.map)):
            for j in range(len(self.map[i])):
                if (i,j) == self.start:
                    distances[(i,j)] = 0
                else:
                    distances[(i,j)] = float('inf')
                previous[(i,j)] = None
                unvisited.append((i,j))
        
        while len(unvisited) > 0:
            raw_distances = list(distances.values())
            if raw_distances.count(float('inf')) == len(raw_distances):
                break
            
            current_distance = min(raw_distances)
            current_index = [i for i in distances if distances[i] == current_distance][0]

            if current_index == self.goal:
                break

            unvisited.remove(current_index)
            del distances[current_index]

            neighbors = self.getNeighbors(current_index, len(self.map), len(self.map[0]))
            
            for neighbor in neighbors:
                if neighbor in unvisited:
                    neighbor_distance = current_distance + 1

                    if neighbor_distance < distances[neighbor]:
                        distances[neighbor] = neighbor_distance
                        previous[neighbor] = current_index
        
        path = [self.goal]
        current_index = self.goal

        while current_index != self.start:
            current_index = previous[current_index]
            path.insert(0, current_index)
        
        return path
            
            
    def getNeighbors(self, current_index, max_i, max_j):
        neighbors = []
        for i in range(current_index[0]-1, current_index[0]+2):
            for j in range(current_index[1]-1, current_index[1]+2):
                if i >= 0 and i < max_i and j >= 0 and j < max_j and not((i,j) == current_index) and self.map[i][j] != self.obstacle_identifier:
                    neighbors.append((i,j))
        return neighbors
    
    def timer_callback(self):
        msg = String()
        path = self.get_path()
        msg.data = f"Goal: {path}"
        self.get_logger().info(f"{msg.data}")   


def map_main():
    rclpy.init()

    # create nodes
    mapNode = MapPublisher()

    # use nodes
    rclpy.spin(mapNode)

    # destroy nodes
    mapNode.destroy_node()

    rclpy.shutdown()

if __name__ == '__map_main__':
    map_main()


def start_main():
    rclpy.init()

    # create nodes
    startNode = StartPublisher()

    # use nodes
    rclpy.spin(startNode)

    # destroy nodes
    startNode.destroy_node()

    rclpy.shutdown()

if __name__ == '__start_main__':
    start_main()


def goal_main():
    rclpy.init()

    # create nodes
    goalNode = GoalPublisher()

    # use nodes
    rclpy.spin(goalNode)

    # destroy nodes
    goalNode.destroy_node()

    rclpy.shutdown()


def path_main():
    rclpy.init()

    # create nodes
    map = MapPublisher()
    start = StartPublisher()
    goal = GoalPublisher()
    pathNode = PathPublisher(map.map, start.start, goal.goal)

    # use nodes
    rclpy.spin(pathNode)

    # destroy nodes
    pathNode.destroy_node()

    rclpy.shutdown()

if __name__ == '__goal_main__':
    goal_main()

path_main()