import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import time
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Sensor Node Class
class SensorNode(Node):

    # Constructor
    def __init__(self):
        super().__init__('sensor_script')

        # subscriber object (to 'go' signal from main) (using 'Transient Local QoS')
        self.subscriber_sensors_start = self.create_subscription(
            Bool, 
            'start_signal', 
            self.evaluate_start_signal, 
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
        )

        # publisher object (to sensor data)
        self.publisher_sensor_data_ = self.create_publisher(
            Float32, 
            'sensor_data',
            10
        )

        self.get_logger().error("Sensor node created!!")

    # start sensor functionality
    def evaluate_start_signal(self, msg):
        
        if msg.data:
            self.get_logger().error("Sensor Script: Checks passed. Starting sensor publishing.")
            
            # timer object (simulated data generation)
            self.timer_simulated_data_ = self.create_timer(1.0, self.timer_callback_generate_data)
        else:
            self.get_logger().error("Sensor Script: Checks failed. Cannot start sensor publishing.")

    # timer callback (simulated data generation)
    def timer_callback_generate_data(self):

        # generate simulated value
        msg = Float32()
        msg.data = 5.0 + (time.time() % 10)

        # publish simulated value
        self.get_logger().info(f"Sensor Script: Publishing sensor data: {msg.data}")
        self.publisher_sensor_data_.publish(msg)
        

# Main function
def main(args=None):
    rclpy.init(args=args)
    
    node = SensorNode()
    #node.wait_main_checks()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()