#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"

class MotionModel : public rclcpp::Node{
    public:
        MotionModel();
        ~MotionModel();
        void step_forward();
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_pose;
        geometry_msgs::msg::Pose pose_composition(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2);
        unsigned int cont;
        geometry_msgs::msg::Pose pose_actual;
};