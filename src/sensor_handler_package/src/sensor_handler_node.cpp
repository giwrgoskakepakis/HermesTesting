#include "sensor_handler_node.hpp"
#include <iostream>
#include <cstdlib>

SensorHandlerNode::SensorHandlerNode() : Node("sensor_handler_node") {
    // Parameter
    this->declare_parameter<std::vector<std::string>>("sensor_names");
    this->sensor_names = this->get_parameter("sensor_names").as_string_array();

    // Subscriber objects (to 'sensor data')
    subscriber_gyroscope_sensor_data_ = this->create_subscription<msg_types::msg::Gyroscope>(
        sensor_names[0] + "_data", 10,
        std::bind(&SensorHandlerNode::gyroscope_sensor_data_updating, this, std::placeholders::_1)
    );

    // Timer object (to handle sensor data periodically)
    timer_handle_data_ = this->create_wall_timer(
        std::chrono::seconds(3),
        std::bind(&SensorHandlerNode::gyroscope_sensor_data_handling, this)
    );
}

// Callback to update sensor data
void SensorHandlerNode::gyroscope_sensor_data_updating(const msg_types::msg::Gyroscope::SharedPtr msg) {
    gyroscope_angles.angle_x = msg->angle_x;
    gyroscope_angles.angle_y = msg->angle_y;
    RCLCPP_INFO(this->get_logger(), ("Updated " + YELLOW + "gyroscope_sensor" + RESET + " data: %f %f").c_str(), gyroscope_angles.angle_x, gyroscope_angles.angle_y);
}

// Callback to handle sensor data periodically
void SensorHandlerNode::gyroscope_sensor_data_handling() {
    RCLCPP_INFO(this->get_logger(), (GREEN + "Handling gyroscope_sensor data periodically: %f %f" + RESET).c_str(), gyroscope_angles.angle_x, gyroscope_angles.angle_y);
}

// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorHandlerNode>());
    rclcpp::shutdown();
    return 0;
}
