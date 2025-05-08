#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

class ArmController : public rclcpp::Node
{
    public:
        ArmController();
        ~ArmController();
        void procesa_laser(const std_msgs::msg::Float32::SharedPtr msg);
        void target_pose(const geometry_msgs::msg::Pose::SharedPtr TargetPose);
        void orden_teclado();
        
        bool teclado;
        float x;
        float y;
        float z;
        geometry_msgs::msg::Pose target;
        geometry_msgs::msg::Pose punto;

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_vel;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_goto;

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_laser;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_pose;
};