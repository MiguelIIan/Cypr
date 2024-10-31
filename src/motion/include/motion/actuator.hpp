#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Actuator : public rclcpp::Node{
    public:
        Actuator();
        ~Actuator();
        void execute_command(const std_msgs::msg::String::SharedPtr msg);
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
};