#include "sensor_handler_node.hpp"
#include <iostream>
#include <cstdlib>

SensorHandlerNode::SensorHandlerNode() : Node("sensor_handler_node") {
    // Parameter
    this->declare_parameter<std::vector<std::string>>("sensor_names");
    this->sensor_names = this->get_parameter("sensor_names").as_string_array();

    // Gyroscope: Subscriber object
    subscriber_gyroscope_sensor_data_ = this->create_subscription<msg_types::msg::Gyroscope>(
        sensor_names[0] + "_data", 10,
        std::bind(&SensorHandlerNode::gyroscope_sensor_data_updating, this, std::placeholders::_1)
    );

    // Temperature: subscriber object
    subscriber_temperatures_sensor_data_ = this->create_subscription<msg_types::msg::Temperature>(
        sensor_names[1] + "_data", 10,
        std::bind(&SensorHandlerNode::temperatures_sensor_data_updating, this, std::placeholders::_1)
    );

    // Timer object for ALL sensors
    timer_handle_data_ = this->create_wall_timer(
        std::chrono::seconds(3),
        std::bind(&SensorHandlerNode::handle_data, this)
    );

}


// ======================================== Updating Functions ========================================
// Gyroscope: Callback to update sensor data
void SensorHandlerNode::gyroscope_sensor_data_updating(const msg_types::msg::Gyroscope::SharedPtr msg) {
    gyroscope_angles.angle_x = msg->angle_x;
    gyroscope_angles.angle_y = msg->angle_y;
    RCLCPP_INFO(this->get_logger(), ("Updated " + YELLOW + "gyroscope_sensor" + RESET + " data: %f %f").c_str(), gyroscope_angles.angle_x, gyroscope_angles.angle_y);
}

// Temperatures: Callback to update sensor data
void SensorHandlerNode::temperatures_sensor_data_updating(const msg_types::msg::Temperature::SharedPtr msg) {
    for (size_t i = 0; i < msg->temperatures_array.size(); ++i) {
        temperatures.temp_values[i] = msg->temperatures_array[i];
    }

    RCLCPP_INFO(this->get_logger(), ("Updated " + BLUE + "temperatures_sensor" + RESET + " data: %d %d %d %d").c_str(), temperatures.temp_values[0], temperatures.temp_values[1], temperatures.temp_values[2], temperatures.temp_values[3]);
}


// ======================================== Handling Functions ========================================
// Gyroscope: data handling
void SensorHandlerNode::gyroscope_sensor_data_handling() {
    RCLCPP_INFO(this->get_logger(), (GREEN + "Handling gyroscope_sensor data periodically: %f %f" + RESET).c_str(), gyroscope_angles.angle_x, gyroscope_angles.angle_y);
}

// Temperatures: data handling
void SensorHandlerNode::temperatures_sensor_data_handling() {
    RCLCPP_INFO(this->get_logger(), (MAGENTA + "Handling temperature data periodically: %d %d %d %d" + RESET).c_str(), temperatures.temp_values[0], temperatures.temp_values[1], temperatures.temp_values[2], temperatures.temp_values[3]);
}

// Callback to handle sensor data periodically
void SensorHandlerNode::handle_data() {
    gyroscope_sensor_data_handling();
    temperatures_sensor_data_handling();
}



// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorHandlerNode>());
    rclcpp::shutdown();
    return 0;
}
