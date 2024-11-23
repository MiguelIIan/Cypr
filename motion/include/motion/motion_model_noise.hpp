#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <string>

class MotionModel : public rclcpp::Node{
    public:
        MotionModel();
        ~MotionModel();
        void step_forward();
        geometry_msgs::msg::Pose generate_noise_sample();
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_pose;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_vector;
        geometry_msgs::msg::Pose pose_composition(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2);
        unsigned int cont;
        geometry_msgs::msg::Pose pose_actual;
        std::vector<geometry_msgs::msg::Pose> estimated_pose_array;
        geometry_msgs::msg::Pose pose_ruido;
        geometry_msgs::msg::Pose p_inc_ruidosa;
        std::string poses_to_json(std::vector<geometry_msgs::msg::Pose> vec_poses);
};