#include "sensor_handler_node.hpp"
#include <iostream>
#include <cstdlib>

SensorHandlerNode::SensorHandlerNode() : Node("sensor_handler_node") {

    // Parameter
    this->declare_parameter<std::vector<std::string>>("sensor_names");
    this->sensor_names = this->get_parameter("sensor_names").as_string_array();

    // ======================================== Subscribers to Sensor Topics ========================================
    // Gyroscope:
    subscriber_gyroscope_sensor_data_ = this->create_subscription<msg_types::msg::Gyroscope>(
        sensor_names[0] + "_data", 10,
        std::bind(&SensorHandlerNode::gyroscope_sensor_data_updating, this, std::placeholders::_1)
    );

    // Temperature:
    subscriber_temperatures_sensor_data_ = this->create_subscription<msg_types::msg::Temperature>(
        sensor_names[1] + "_data", 10,
        std::bind(&SensorHandlerNode::temperatures_sensor_data_updating, this, std::placeholders::_1)
    );

    // Voltage:
    subscriber_voltages_sensor_data_ = this->create_subscription<msg_types::msg::Voltage>(
        sensor_names[2] + "_data", 10,
        std::bind(&SensorHandlerNode::voltages_sensor_data_updating, this, std::placeholders::_1)
    );

    // Sole Pressure
    subscriber_sole_pressures_sensor_data_ = this->create_subscription<msg_types::msg::SolePressure>(
        sensor_names[3] + "_data", 10,
        std::bind(&SensorHandlerNode::sole_pressures_sensor_data_updating, this, std::placeholders::_1)
    );

    // Timer object for ALL sensors
    timer_handle_data_ = this->create_wall_timer(
        std::chrono::seconds(3),
        std::bind(&SensorHandlerNode::handle_data, this)
    );

}


// ======================================== Updating Functions ========================================
// Gyroscope:
void SensorHandlerNode::gyroscope_sensor_data_updating(const msg_types::msg::Gyroscope::SharedPtr msg) {
    gyroscope_angles.angle_x = msg->angle_x;
    gyroscope_angles.angle_y = msg->angle_y;
    RCLCPP_INFO(this->get_logger(), ("Updated " + YELLOW + "gyroscope_sensor" + RESET + " data: %f %f").c_str(), gyroscope_angles.angle_x, gyroscope_angles.angle_y);
}

// Temperatures:
void SensorHandlerNode::temperatures_sensor_data_updating(const msg_types::msg::Temperature::SharedPtr msg) {
    for (size_t i = 0; i < msg->temperatures_array.size(); ++i) {
        temperatures.temp_values[i] = msg->temperatures_array[i];
    }

    RCLCPP_INFO(this->get_logger(), ("Updated " + BLUE + "temperatures_sensor" + RESET + " data: %d %d %d %d").c_str(), temperatures.temp_values[0], temperatures.temp_values[1], temperatures.temp_values[2], temperatures.temp_values[3]);
}

// Voltages:
void SensorHandlerNode::voltages_sensor_data_updating(const msg_types::msg::Voltage::SharedPtr msg) {
    for (size_t i = 0; i < msg->voltages_array.size(); ++i) {
        voltages.volt_values[i] = msg->voltages_array[i];
    }

    RCLCPP_INFO(this->get_logger(), ("Updated " + CYAN + "voltages_sensor" + RESET + " data: %f %f %f %f").c_str(), voltages.volt_values[0], voltages.volt_values[1], voltages.volt_values[2], voltages.volt_values[3]);
}

// Sole Pressures
void SensorHandlerNode::sole_pressures_sensor_data_updating(const msg_types::msg::SolePressure::SharedPtr msg) {
    for (size_t i = 0; i < msg->left_sole_array.size(); ++i) {
        sole_pressures.left_sole_values[i] = msg->left_sole_array[i];
        sole_pressures.right_sole_values[i] = msg->right_sole_array[i];
    }

    RCLCPP_INFO(this->get_logger(), ("Updated " + MAGENTA + "sole_pressures_sensor" + RESET + " data: %f %f %f %f").c_str(), sole_pressures.left_sole_values[0], sole_pressures.left_sole_values[1], sole_pressures.right_sole_values[0], sole_pressures.right_sole_values[1]);
}

// ======================================= Handling Functions ========================================
// Gyroscope:
void SensorHandlerNode::gyroscope_sensor_data_handling() {
    RCLCPP_INFO(this->get_logger(), (GREEN + "Handling gyroscope_sensor data periodically: %f %f" + RESET).c_str(), gyroscope_angles.angle_x, gyroscope_angles.angle_y);
}

// Temperatures:
void SensorHandlerNode::temperatures_sensor_data_handling() {
    RCLCPP_INFO(this->get_logger(), (GREEN + "Handling temperature data periodically: %d %d %d %d" + RESET).c_str(), temperatures.temp_values[0], temperatures.temp_values[1], temperatures.temp_values[2], temperatures.temp_values[3]);
}

// Voltages:
void SensorHandlerNode::voltages_sensor_data_handling() {
    RCLCPP_INFO(this->get_logger(), (GREEN + "Handling voltage data periodically: %f %f %f %f" + RESET).c_str(), voltages.volt_values[0], voltages.volt_values[1], voltages.volt_values[2], voltages.volt_values[3]);
}

// Sole Pressures:
void SensorHandlerNode::sole_pressures_sensor_data_handling() {
    RCLCPP_INFO(this->get_logger(), (GREEN + "Handling sole pressure data periodically: %f %f %f %f" + RESET).c_str(), sole_pressures.left_sole_values[0], sole_pressures.left_sole_values[1], sole_pressures.right_sole_values[0], sole_pressures.right_sole_values[1]);
}

// Callback to handle sensor data periodically
void SensorHandlerNode::handle_data() {
    gyroscope_sensor_data_handling();
    temperatures_sensor_data_handling();
    voltages_sensor_data_handling();
    sole_pressures_sensor_data_handling();
}


// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorHandlerNode>());
    rclcpp::shutdown();
    return 0;
}
