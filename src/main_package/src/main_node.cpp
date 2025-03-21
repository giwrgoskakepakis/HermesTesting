#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <iostream>
#include <cstdlib>

using namespace std::chrono_literals;

const std::string GREEN = "\033[92m";
const std::string RED = "\033[91m";
const std::string RESET = "\033[0m";

// Main Node Class
class MainNode : public rclcpp::Node {
    public:

        // Constructor
        MainNode() : Node("main_node") {
        
            // Parameter (sensor names)
            this->declare_parameter<std::vector<std::string>>("sensor_names");
            this->sensor_names = this->get_parameter("sensor_names").as_string_array();

            // signal to start sensors after checks (Latching)
            publisher_sensors_start_ = this->create_publisher<std_msgs::msg::Bool>(
                "sensors_start_signal", rclcpp::QoS(1).transient_local()
            );
        }

        // Simulated sensor checks
        void init_electronics() {
            auto msg = std_msgs::msg::Bool();

            // simulated checks
            bool sensor_check = check_sensor_status();
            bool config_check = check_configuration();

            if (sensor_check && config_check) {
                RCLCPP_INFO(this->get_logger(), (GREEN + "        Main Node: Init Electronics passed successfully." + RESET).c_str());
                msg.data = true;
            }
            else {
                RCLCPP_ERROR(this->get_logger(), (RED + "        Main Node: Init Electronics failed." + RESET).c_str());
                msg.data = false;
            }

            // send the signal
            publisher_sensors_start_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Sent start signal!!!");
        }

    private:
        
        // dummy checks
        bool check_sensor_status() { return true; }
        bool check_configuration() { return true; }

        // attributes
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_sensors_start_;
        std::vector<std::string> sensor_names;
};


// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Create the node
    auto main_node = std::make_shared<MainNode>();  

    // Perform sensor checks
    main_node->init_electronics();

    rclcpp::spin(main_node);
    rclcpp::shutdown();
    return 0;
}
