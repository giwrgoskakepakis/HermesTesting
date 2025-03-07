import random
import rclpy
from msg_types.msg import Gyroscope
from sensor_package.sensor_node import SensorNode

# Sensor Node Class
class GyroscopeSensorNode(SensorNode):

    # Constructor
    def __init__(self):
        super().__init__('gyroscope_sensor', Gyroscope, 0.5, [])

    # interface implementation
    def take_measurements(self):

        msg =  Gyroscope()
        msg.angle_x = random.uniform(0.0, 100.0)
        msg.angle_y = random.uniform(0.0, 100.0)

        return msg

# Main function
def main(args=None):
    rclpy.init(args=args)
    
    node = GyroscopeSensorNode()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()