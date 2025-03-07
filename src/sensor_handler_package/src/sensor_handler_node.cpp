#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp" 
#include <msg_types/msg/temperature.hpp>
#include <iostream>
#include <cstdlib>

using namespace std::chrono_literals;

const std::string GREEN = "\033[92m";
const std::string RED = "\033[91m";
const std::string RESET = "\033[0m";
const std::string YELLOW = "\033[93m";
const std::string BLUE = "\033[94m";

// Sensor Handler Class
class SensorHandlerNode : public rclcpp::Node {
    public:

        // Constructor
        SensorHandlerNode() : Node("sensor_handler_node") {

            // Parameter
            this->declare_parameter<std::vector<std::string>>("sensor_names");
            this->sensor_names = this->get_parameter("sensor_names").as_string_array();

            // subscriber objects (to 'sensor data')
            subscriber_sensor1_data_ = this->create_subscription<msg_types::msg::Temperature>(
                sensor_names[0] + "_data", 10,
                std::bind(&SensorHandlerNode::sensor1_data_updating, this, std::placeholders::_1)
            );

            subscriber_sensor2_data_ = this->create_subscription<std_msgs::msg::String>(
                sensor_names[1] + "_data", 10,
                std::bind(&SensorHandlerNode::sensor2_data_updating, this, std::placeholders::_1)
            );

            // timer object (to hanlde sensor data periodically)
            timer_handle_data_ = this->create_wall_timer(
                std::chrono::seconds(3), 
                std::bind(&SensorHandlerNode::sensor1_data_handling, this)
            );
        }

    private:

        // timer callback (just update the data)
        void sensor1_data_updating(const msg_types::msg::Temperature::SharedPtr msg) {
            latest_sensor1_data_ = msg->var2;
            latest_sensor1_error_codes = msg->var1;
            RCLCPP_INFO(this->get_logger(), ("Updated " + YELLOW + "sensor1" + RESET + " data: %f %d").c_str(), latest_sensor1_data_, latest_sensor1_error_codes);
        }

        void sensor2_data_updating(const std::shared_ptr<std_msgs::msg::String> msg) {
            latest_sensor2_data_ = msg->data;
            RCLCPP_INFO(this->get_logger(), ("Updated " + BLUE + "sensor2" + RESET + " data: %s").c_str(), latest_sensor2_data_.c_str());
        }

        // subscriber callback (after receiving sensor data)
        void sensor1_data_handling() {
            RCLCPP_INFO(this->get_logger(), (GREEN + "Handling sensor1 data periodically: %f %d" + RESET).c_str(), latest_sensor1_data_, latest_sensor1_error_codes);
        }
    
        // Attributes
        rclcpp::Subscription<msg_types::msg::Temperature>::SharedPtr subscriber_sensor1_data_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_sensor2_data_;
        rclcpp::TimerBase::SharedPtr timer_handle_data_;
        float latest_sensor1_data_ = 0.0f;
        int latest_sensor1_error_codes = 0;
        std::string latest_sensor2_data_ = "";
        std::vector<std::string> sensor_names;
};

// Main function
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorHandlerNode>());
    rclcpp::shutdown();
    return 0;
}