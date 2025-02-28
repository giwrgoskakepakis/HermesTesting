import rclpy
from std_msgs.msg import String
import time

from sensor_package.sensor_node import SensorNode

YELLOW = "\033[93m" 
BLUE = "\033[94m"
RESET = "\033[0m" 

# Sensor Node Class
class Sensor2Node(SensorNode):

    # Constructor
    def __init__(self):
        super().__init__('sensor2_script', String, 0.5, ["randomstring"])

    # interface implementation
    def take_measurements(self, x):

        msg =  String()
        msg.data =  str(time.time() % 10)

        return msg
        

# Main function
def main(args=None):
    rclpy.init(args=args)
    
    node = Sensor2Node()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()