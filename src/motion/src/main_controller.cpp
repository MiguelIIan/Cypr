#include <motion/controller.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    Controller p;
    rclcpp::Rate loop_rate(1);
    while (rclcpp::ok()){
        p.publish_method();
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
