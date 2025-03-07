#ifndef SENSOR_HANDLER_NODE_HPP
#define SENSOR_HANDLER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <msg_types/msg/gyroscope.hpp>
#include <msg_types/msg/temperature.hpp>
#include <msg_types/msg/voltage.hpp>
#include <msg_types/msg/sole_pressure.hpp>
#include <vector>
#include <string>

using namespace std::chrono_literals;

struct Temperatures{
    std::vector<int> temp_values; 
    Temperatures() : temp_values(15, 0.0f) {}
};

struct Voltages {
    std::vector<float> volt_values;
    Voltages() : volt_values(15, 0.0f) {}
};

struct SolePressures{
    std::vector<float> left_sole_values; 
    std::vector<float> right_sole_values; 
    SolePressures() : left_sole_values(15, 0.0f), right_sole_values(15, 0.0f) {}
};

struct GyroscopeAngles{
    float angle_x;
    float angle_y;
    GyroscopeAngles() : angle_x(0.0f), angle_y(0.0f) {}
};

const std::string GREEN = "\033[92m";
const std::string RED = "\033[91m";
const std::string RESET = "\033[0m";
const std::string YELLOW = "\033[93m";
const std::string BLUE = "\033[94m";
const std::string MAGENTA = "\033[95m";
const std::string CYAN = "\033[96m";

class SensorHandlerNode : public rclcpp::Node {
    public:
        SensorHandlerNode();

    private:
        void gyroscope_sensor_data_updating(const msg_types::msg::Gyroscope::SharedPtr msg);
        void temperatures_sensor_data_updating(const msg_types::msg::Temperature::SharedPtr msg);
        void voltages_sensor_data_updating(const msg_types::msg::Voltage::SharedPtr msg);
        void sole_pressures_sensor_data_updating(const msg_types::msg::SolePressure::SharedPtr msg);
        
        void handle_data();
        void gyroscope_sensor_data_handling();
        void temperatures_sensor_data_handling();
        void voltages_sensor_data_handling();
        void sole_pressures_sensor_data_handling();


        rclcpp::Subscription<msg_types::msg::Gyroscope>::SharedPtr subscriber_gyroscope_sensor_data_;
        rclcpp::Subscription<msg_types::msg::Temperature>::SharedPtr subscriber_temperatures_sensor_data_;
        rclcpp::Subscription<msg_types::msg::Voltage>::SharedPtr subscriber_voltages_sensor_data_;
        rclcpp::Subscription<msg_types::msg::SolePressure>::SharedPtr subscriber_sole_pressures_sensor_data_;

        rclcpp::TimerBase::SharedPtr timer_handle_data_;

        std::vector<std::string> sensor_names;
        GyroscopeAngles gyroscope_angles;
        Temperatures temperatures;
        Voltages voltages;
        SolePressures sole_pressures;
};

#endif // SENSOR_HANDLER_NODE_HPP
