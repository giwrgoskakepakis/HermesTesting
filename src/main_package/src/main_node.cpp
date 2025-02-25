#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <iostream>
#include <cstdlib>
#include <thread>

// new commend

using namespace std::chrono_literals;

const std::string GREEN = "\033[92m";
const std::string RED = "\033[91m";
const std::string RESET = "\033[0m";

// Main Node Class
class MainNode : public rclcpp::Node {
    public:

        // Constructor
        MainNode() : Node("main_node") {

            // Create the service
            checks_service_ = this->create_service<std_srvs::srv::Trigger>(
                "check_status", 
                std::bind(&MainNode::handle_request, this, std::placeholders::_1, std::placeholders::_2)
            );
        }

    private:
        
        // dummy checks
        bool check_sensor_status() { return true; }
        bool check_configuration() { return true; }

        // Simulated checks
        bool perform_checks() {

            // simulated checks
            bool sensor_check = check_sensor_status();
            bool config_check = check_configuration();

            if (sensor_check && config_check) {
                RCLCPP_INFO(this->get_logger(), (GREEN + "        Main Node: Checks passed successfully." + RESET).c_str());
                return true;
            }
            else {
                RCLCPP_ERROR(this->get_logger(), (RED + "        Main Node: Checks failed." + RESET).c_str());
                return false;
            }
        }

        // Subscriber callback
        void subscriber_callback(const std_msgs::msg::Float32::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "        Main Node:     Received sensor data:   %f", msg->data);
        }

        // Request callback
        void handle_request(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            bool res;

            RCLCPP_INFO(this->get_logger(), "        Main Node:     Received request to check status.");

            // Simulated checks
            RCLCPP_INFO(this->get_logger(), "        Main Node: Performing checks...");
            res = perform_checks();
            RCLCPP_INFO(this->get_logger(), "        Main Node: Checks completed.");

            // Send the response
            response->success = res;
            response->message = "Checks are completed. Python can proceed.";

            // If checks passed successfully / failed
            if (res) {

                // Subscriber object
                subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
                    "/sensor_data", 10, std::bind(&MainNode::subscriber_callback, this, std::placeholders::_1)
                );
                RCLCPP_INFO(this->get_logger(), "        Main Node: Waiting for sensor data...");
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "        Main Node:Simulated checks failed. Python script will not run.");
            }

        }

        // attribute
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr checks_service_;
};


// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainNode>());
    rclcpp::shutdown();
    return 0;
}
