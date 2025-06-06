import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class SimulaPlanningNode(Node):
    def __init__(self):
        super().__init__('simula_planning_node')
        self.publisher = self.create_publisher(Float64MultiArray, 'planned_path', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        simulated_path = np.array([
            [0.0, -1.0],
            [0.1, -0.9],
            [0.2, -0.8],
            [0.3, -0.75],
            [0.4, -0.7]
        ], dtype=np.float64)

        msg = Float64MultiArray()
        msg.data = simulated_path.flatten().tolist()
        self.publisher.publish(msg)
        self.get_logger().info('Publicando caminho simulado.')

def main(args=None):
    rclpy.init(args=args)
    node = SimulaPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
  
