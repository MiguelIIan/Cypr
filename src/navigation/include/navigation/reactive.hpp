#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

class Reactivo : public rclcpp::Node
{
    public:
        Reactivo();
        ~Reactivo();
        void proximo_paso();
        void process_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void estimated_pose(const geometry_msgs::msg::Pose::SharedPtr groundtruth);
        void odom(const nav_msgs::msg::Odometry::SharedPtr odometria);
        int pos_in_array;
        double nearest_obstacle_distance;
        geometry_msgs::msg::Twist movimiento;
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_pose;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
};
