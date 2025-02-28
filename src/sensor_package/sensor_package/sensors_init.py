# This could be the entry point of sensors system. 
# All sensor nodes are spinned from here
import rclpy
import rclpy.executors
from sensor_package.sensor1_script import Sensor1Node
from sensor_package.sensor2_script import Sensor2Node

def main(args=None):
    rclpy.init(args=args)

    sensor1 = Sensor1Node()
    sensor2 = Sensor2Node()

    executor = rclpy.executors.MultiThreadedExecutor()

    executor.add_node(sensor1)
    executor.add_node(sensor2)
    
    try: 
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        sensor1.destroy_node()
        sensor2.destroy_node()
        rclpy.shutdown()

