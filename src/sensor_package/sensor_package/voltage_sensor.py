import random
import rclpy
from msg_types.msg import Voltage
from sensor_package.sensor_node import SensorNode

# Sensor Node Class
class VoltageSensorNode(SensorNode):

    # Constructor
    def __init__(self):
        super().__init__('voltage_sensor', Voltage, 0.5, [])

    # interface implementation
    def take_measurements(self):
        msg =  Voltage()
        msg.voltages_array = [random.uniform(0.0, 100.0) for _ in range(15)]
        return msg

# Main function
def main(args=None):
    rclpy.init(args=args)
    
    node = VoltageSensorNode()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()