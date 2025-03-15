#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>
#include "rclcpp/qos.hpp"

using namespace std::chrono_literals;

class HandlerNode : public rclcpp_lifecycle::LifecycleNode {
public:
    HandlerNode() : LifecycleNode("handler_node") {}


    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Configuring handler node...");
        subscription_ = create_subscription<std_msgs::msg::String>(
            "sensor_data", 10, std::bind(&HandlerNode::data_callback, this, std::placeholders::_1));
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Activating handler node...");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Deactivating handler node...");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    void data_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Received: %s", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HandlerNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}