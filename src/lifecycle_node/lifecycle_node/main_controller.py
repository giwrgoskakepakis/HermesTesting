import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from rclpy.lifecycle import TransitionCallbackReturn, LifecycleState

import time

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')

        # Clients for triggering lifecycle transitions
        self.sensor_client = self.create_client(ChangeState, '/sensor_node/change_state')
        self.handler_client = self.create_client(ChangeState, '/sensor_handler/change_state')

        self.get_logger().info("Waiting for lifecycle services...")

        self.sensor_client.wait_for_service()
        self.handler_client.wait_for_service()

        self.get_logger().info("Services available. Configuring nodes...")
        
        # Configure both nodes
        self.configure_node(self.sensor_client)
        self.configure_node(self.handler_client)

        # Activate both nodes
        self.activate_node(self.sensor_client)
        self.activate_node(self.handler_client)

        self.get_logger().info("All nodes activated!")

    def configure_node(self, client):
        """Request lifecycle transition to CONFIGURE state"""
        request = ChangeState.Request()
        request.transition.id = TransitionCallbackReturn.TRANSITION_CONFIGURE
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f"Node configured successfully")
        else:
            self.get_logger().error(f"Failed to configure node")

    def activate_node(self, client):
        """Request lifecycle transition to ACTIVATE state"""
        request = ChangeState.Request()
        request.transition.id = TransitionCallbackReturn.TRANSITION_ACTIVATE
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f"Node activated successfully")
        else:
            self.get_logger().error(f"Failed to activate node")

def main(args=None):
    rclpy.init(args=args)
    node = MainController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
