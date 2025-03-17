import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from std_msgs.msg import String

class SensorNode(LifecycleNode):
    def __init__(self):
        super().__init__('sensor_node')
        self.publisher = None
        self.timer = None
        self.shutdown_requested = False  # This flag will be set in on_shutdown()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring sensor node...")
        self.publisher = self.create_lifecycle_publisher(String, 'sensor_data', 10)
        self.timer = self.create_timer(1.0, self.publish_data)
        self.timer.cancel()  # Timer is initially cancelled
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating sensor node...")
        self.timer.reset()  # Restart the timer when activated
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().error("Error detected. Transitioning to error state")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating sensor node...")
        self.timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up sensor node...")
        self._destroy_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Preparing sensor node for shutdown...")
        self._destroy_resources()
        # Instead of raising an exception or calling rclpy.shutdown() here,
        # set a flag to signal that shutdown is desired.
        self.shutdown_requested = True
        return TransitionCallbackReturn.SUCCESS

    def _destroy_resources(self):
        if self.publisher:
            self.destroy_lifecycle_publisher(self.publisher)
            self.publisher = None
        if self.timer:
            self.destroy_timer(self.timer)
            self.timer = None

    def publish_data(self):
        msg = String()
        msg.data = "Hello world"
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()

    # Instead of rclpy.spin(node), use a loop that checks the shutdown flag.
    while rclpy.ok() and not node.shutdown_requested:
        rclpy.spin_once(node, timeout_sec=0.5)

    node.get_logger().info("Node shut down successfully. Exiting process...")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
