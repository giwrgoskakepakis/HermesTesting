#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <iostream>
#include <cstdlib>
#include <thread>

// new comment

using namespace std::chrono_literals;

const std::string GREEN = "\033[92m";
const std::string RED = "\033[91m";
const std::string RESET = "\033[0m";

// Main Node Class
class MainNode : public rclcpp::Node {
    public:

        // Constructor
        MainNode() : Node("main_node") {
        
            // signal to start sensors after checks (Latching)
            publisher_sensors_start_ = this->create_publisher<std_msgs::msg::Bool>(
                "sensors_start_signal", rclcpp::QoS(1).transient_local()
            );
        }

        // Simulated sensor checks
        void perform_checks() {
            auto msg = std_msgs::msg::Bool();

            // simulated checks
            bool sensor_check = check_sensor_status();
            bool config_check = check_configuration();

            if (sensor_check && config_check) {
                RCLCPP_INFO(this->get_logger(), (GREEN + "        Main Node: Checks passed successfully." + RESET).c_str());
                msg.data = true;
            }
            else {
                RCLCPP_ERROR(this->get_logger(), (RED + "        Main Node: Checks failed." + RESET).c_str());
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
};


// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Create the node
    auto main_node = std::make_shared<MainNode>();  

    // Perform sensor checks
    main_node->perform_checks();

    rclcpp::spin(main_node);
    rclcpp::shutdown();
    return 0;
}
