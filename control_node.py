import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np
import time

from tracking import tracking  # só tracking mesmo

class Control(Node):

    def __init__(self):
        super().__init__('control')
        self.publisher = self.create_publisher(Float64MultiArray, 'control', 1)
        
        self.is_safe = True
        self.safety_subscription = self.create_subscription(
            Bool,
            'safety',
            self.safety_callback,
            1)
        
        self.path_subscription = self.create_subscription(
            Float64MultiArray,
            'planned_path',  # o tópico que simula_planning_node publica
            self.path_callback,
            1)
        
        self.prev_time = time.time()
        self.current_speed = 0.0  # inicializa a velocidade atual (pode melhorar depois)
        
    def safety_callback(self, msg):
        self.is_safe = msg.data

    def path_callback(self, msg):
        path = np.array(msg.data).reshape(-1, 2)
        
        # calcular dt
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        
        speed = 5.0  # velocidade desejada fixa ou pode fazer controle depois
        
        if self.is_safe:
            steer, pedal = tracking(path, speed, 0.0, dt)
        else:
            steer, pedal = 0.0, 0.0
        
        # publicar comando
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [steer, pedal]
        self.publisher.publish(cmd_msg)
        
        self.get_logger().info(f'Steer: {steer:.3f}, Pedal: {pedal:.3f}, dt: {dt:.3f}, Safe: {self.is_safe}')
        

def main(args=None):
    rclpy.init(args=args)
    control_node = Control()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
