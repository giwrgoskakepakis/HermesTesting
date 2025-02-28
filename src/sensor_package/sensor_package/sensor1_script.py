from std_msgs.msg import Float32
import time
import random
from msg_types.msg import Temperature

from sensor_package.sensor_node import SensorNode

YELLOW = "\033[93m" 
BLUE = "\033[94m"
RESET = "\033[0m" 

# Sensor Node Class
class Sensor1Node(SensorNode):

    # Constructor
    def __init__(self):
        super().__init__('sensor1_script', Temperature, 0.5, [1])

    # interface implementation
    def take_measurements(self, x):

        msg =  Temperature()
        msg.var2 = random.uniform(0.0, 100.0)
        msg.var1 = random.randint(0, 3)
        # msg.data =  x + (time.time() % 10)

        return msg

# Main function
# def main(args=None):
#     rclpy.init(args=args)
    
#     node = Sensor1Node()
    
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()