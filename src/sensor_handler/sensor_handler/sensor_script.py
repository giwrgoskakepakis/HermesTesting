import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
from std_srvs.srv import Trigger

# Sensor Node Class
class SensorNode(Node):

    # Constructor
    def __init__(self):
        super().__init__('sensor_script')

        # publisher object
        self.publisher_ = self.create_publisher(Float32, 'sensor_data', 10)

        # client object
        self.client_ = self.create_client(Trigger, 'check_status')

    # function that: blocks the node until main checks run successfully
    def wait_main_checks(self):

        # Wait for the service to be available
        self.get_logger().info('Sensor Script: Waiting for the service to be available...')
        self.client_.wait_for_service()

        # Send the request
        self.get_logger().info('Sensor Script: Sending request to check status...')
        request = Trigger.Request()
        future = self.client_.call_async(request)

        # Block until the response is received
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f"Sensor Script: Response from C++ node: {future.result().message}")

                # Start the sensor publishing process
                self.start_publishing()
            else:
                self.get_logger().error("Sensor Script: Checks failed. Cannot start sensor publishing.")
        else:
            self.get_logger().error("Sensor Script: Service call failed.")

    # function that: creates the publishing timer    
    def start_publishing(self):
        self.get_logger().info("Sensor Script: Starting sensor publishing...\n")
        
        # timer object
        self.timer_ = self.create_timer(1.0, self.timer_callback)

    # timer callback
    def timer_callback(self):

        # get simulated value
        msg = Float32()
        msg.data = 5.0 + (time.time() % 10)

        # publish simulated value
        self.get_logger().info(f"Sensor Script: Publishing sensor data: {msg.data}")
        self.publisher_.publish(msg)
        

# Main function
def main(args=None):
    rclpy.init(args=args)
    
    node = SensorNode()
    node.wait_main_checks()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()