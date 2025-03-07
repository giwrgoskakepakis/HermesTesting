import random
import rclpy
from msg_types.msg import SolePressure
from sensor_package.sensor_node import SensorNode

# Sensor Node Class
class SolePressureSensorNode(SensorNode):

    # Constructor
    def __init__(self):
        super().__init__('sole_pressure_sensor', SolePressure, 0.5, [])

    # interface implementation
    def take_measurements(self):
        msg =  SolePressure()
        msg.left_sole_array = [random.uniform(0.0, 100.0) for _ in range(15)]
        msg.right_sole_array = [random.uniform(0.0, 100.0) for _ in range(15)]
        return msg

# Main function
def main(args=None):
    rclpy.init(args=args)
    
    node = SolePressureSensorNode()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()