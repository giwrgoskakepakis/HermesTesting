from abc import ABC, abstractmethod
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class SensorNode(Node, ABC):

    def __init__(self, sensor_name: str, data_type: type, delay: float, args: list):
        super().__init__(sensor_name)

        self.declare_parameter("sensor_name", "default_topic")
        self.sensor_name = self.get_parameter("sensor_name").value
        self.get_logger().info(f"Publishing on topic: {self.sensor_name}")

        self.args = args
        self.delay = delay

        # subscriber object to a one-time topic to wait for main app to finish startup checks before initiating execution
        # even if main app sends the signal before this subscription mounts, it will still hear it
        self.start_signal_subscriber = self.create_subscription(
            Bool, 
            'sensors_start_signal', 
            self.evaluate_start_signal, 
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
        )

        # publisher object (to sensor data)
        self.publisher_sensor_data_ = self.create_publisher(
            data_type, 
            f"{self.sensor_name}_data",
            10
        )

        self.get_logger().info(f"{self.sensor_name} node created successfully")

    def evaluate_start_signal(self, msg):
        
        if msg.data:
            self.get_logger().info(f"{self.sensor_name}: Main signaled OK: Starting sensor publishing.")
            
            # timer object to take measurements
            self.timer_simulated_data_ = self.create_timer(self.delay, self.send_data)
        else:
            self.get_logger().error(f"{self.sensor_name}: Main signaled NOT OK: Cannot start sensor publishing.")

    def send_data(self):
        msg = self.take_measurements(*self.args)

        try: 
            # self.get_logger().info(f"{self.sensor_name}: Publishing sensor data: {msg.data}")
            self.publisher_sensor_data_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"{self.sensor_name}: Publishing sensor data threw an exception: \n {e}")
    
    @abstractmethod
    def take_measurements(self, *args):
        """Abstract method that must be implemented by child classes"""
        pass