import random
import rclpy
from msg_types.msg import Temperature
from sensor_package.sensor_node import SensorNode

# Sensor Node Class
class TemperatureSensorNode(SensorNode):

    # Constructor
    def __init__(self):
        super().__init__('temperature_sensor', Temperature, 0.5, [])

    # interface implementation
    def take_measurements(self):
        msg =  Temperature()
        msg.temperatures_array = [int(random.uniform(0.0, 100.0)) for _ in range(15)]
        return msg

# Main function
def main(args=None):
    rclpy.init(args=args)
    
    node = TemperatureSensorNode()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()