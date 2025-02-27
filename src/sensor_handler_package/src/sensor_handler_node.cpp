#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp" 
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
            this->declare_parameter("test_param", "bye");

            // subscriber objects (to 'sensor data')
            subscriber_sensor1_data_ = this->create_subscription<std_msgs::msg::Float32>(
                "sensor1_script_data", 10,
                std::bind(&SensorHandlerNode::sensor1_data_updating, this, std::placeholders::_1)
            );

            subscriber_sensor2_data_ = this->create_subscription<std_msgs::msg::String>(
                "sensor2_script_data", 10,
                std::bind(&SensorHandlerNode::sensor2_data_updating, this, std::placeholders::_1)
            );

            // timer object (to hanlde sensor data periodically)
            timer_handle_data_ = this->create_wall_timer(
                std::chrono::seconds(3), 
                std::bind(&SensorHandlerNode::sensor1_data_handling, this)
            );

            std::string my_param = this->get_parameter("test_param").as_string();
            RCLCPP_INFO(this->get_logger(), "parameter: %s", my_param.c_str());
        }

    private:

        // timer callback (just update the data)
        void sensor1_data_updating(const std_msgs::msg::Float32::SharedPtr msg) {
            latest_sensor1_data_ = msg->data;
            RCLCPP_INFO(this->get_logger(), ("Updated " + YELLOW + "sensor1" + RESET + " data: %f").c_str(), latest_sensor1_data_);
        }

        void sensor2_data_updating(const std::shared_ptr<std_msgs::msg::String> msg) {
            latest_sensor2_data_ = msg->data;
            RCLCPP_INFO(this->get_logger(), ("Updated " + BLUE + "sensor2" + RESET + " data: %s").c_str(), latest_sensor2_data_.c_str());
        }

        // subscriber callback (after receiving sensor data)
        void sensor1_data_handling() {
            RCLCPP_INFO(this->get_logger(), (GREEN + "Handling sensor1 data periodically: %f" + RESET).c_str(), latest_sensor1_data_);
        }
    
        // Attributes
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_sensor1_data_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_sensor2_data_;
        rclcpp::TimerBase::SharedPtr timer_handle_data_;
        float latest_sensor1_data_ = 0.0f;
        std::string latest_sensor2_data_ = "";
};

// Main function
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorHandlerNode>());
    rclcpp::shutdown();
    return 0;
}