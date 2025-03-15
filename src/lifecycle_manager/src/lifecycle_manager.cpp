#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <chrono>

using namespace std::chrono_literals;

class MainNode : public rclcpp::Node {
public:
    MainNode() : Node("main_node") {
        // Create clients for sensor and handler nodes
        sensor_change_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>("sensor_node/change_state");
        handler_change_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>("handler_node/change_state");

        sensor_get_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>("sensor_node/get_state");
        handler_get_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>("handler_node/get_state");

        // // Timer to manage state transitions
        // timer_ = this->create_wall_timer(1s, std::bind(&MainNode::manage_states, this));
    }


    void manage_states() {
        // timer_->cancel();
        // Dummy checks
        RCLCPP_INFO(this->get_logger(), "Performing dummy checks...");

        // Trigger transition to inactive
        if (!trigger_transition(sensor_change_client_, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure sensor node.");
            return;
        }
        if (!trigger_transition(handler_change_client_, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure handler node.");
            return;
        }

        

        // Check if both nodes are inactive
        uint8_t sensor_state = get_state(sensor_get_state_client_);
        uint8_t handler_state = get_state(handler_get_state_client_);

        if (sensor_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
            handler_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
            RCLCPP_INFO(this->get_logger(), "Both nodes are inactive. Transitioning to active...");
            if (!trigger_transition(sensor_change_client_, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to activate sensor node.");
                return;
            }
            if (!trigger_transition(handler_change_client_, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to activate handler node.");
                return;
            }

        } else {
            RCLCPP_INFO(this->get_logger(), "Nodes are not yet inactive. Sensor state: %d, Handler state: %d", sensor_state, handler_state);
        }
    }

    bool trigger_transition(rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client, uint8_t transition_id) {
        if (!client->wait_for_service(2s)) {
            RCLCPP_ERROR(this->get_logger(), "Service %s not available.", client->get_service_name());
            return false;
        }

        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition_id;
        auto future = client->async_send_request(request);

        // Use a separate executor to wait for the future
        // rclcpp::executors::SingleThreadedExecutor executor;
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Service call to %s failed.", client->get_service_name());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Transition to configure successful");

        return true;
    }

    uint8_t get_state(rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client) {
        if (!client->wait_for_service(2s)) {
            RCLCPP_ERROR(this->get_logger(), "Service %s not available.", client->get_service_name());
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto future = client->async_send_request(request);

        // Use a separate executor to wait for the future
        // rclcpp::executors::SingleThreadedExecutor executor;
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Service call to %s failed.", client->get_service_name());
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        return future.get()->current_state.id;
    }

    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr sensor_change_client_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr handler_change_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr sensor_get_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr handler_get_state_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MainNode>();
    node->manage_states();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}