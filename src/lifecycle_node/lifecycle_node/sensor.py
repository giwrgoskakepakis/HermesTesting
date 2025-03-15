import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.lifecycle import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class SensorNode(LifecycleNode):
    def __init__(self):
        super().__init__('sensor_node')
        self.publisher = None
        self.timer = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring sensor node...")
        
        # Create the lifecycle publisher with explicit QoS settings
        # self.publisher = self.create_publisher(String, 'sensor_data', 10)
        self.publisher = self.create_lifecycle_publisher(String, 'sensor_data', 10)
        self.get_logger().info("Lifecycle publisher created with QoS settings.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating sensor node...")

        self.timer = self.create_timer(1.0, self.publish_data)
        self.get_logger().info("Timer created. Publishing data...")

        return super().on_activate(state)
    
    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().error("ERROR ERROR ERROR")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating sensor node...")
        
        # Destroy the timer
        if self.timer:
            self.timer.destroy()
            self.timer = None
            self.get_logger().info("Timer destroyed.")
        return TransitionCallbackReturn.SUCCESS

    def publish_data(self):
        msg = String()
        msg.data = "Sensor Data"
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()