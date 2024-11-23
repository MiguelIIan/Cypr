#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Controller : public rclcpp::Node{
    public:
        Controller();
        ~Controller();
        void publish_method();
    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
        unsigned int cont;
};