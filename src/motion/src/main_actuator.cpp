#include <motion/actuator.hpp>


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto mi_nodo = std::make_shared<Actuator>();
    rclcpp::spin(mi_nodo);
    rclcpp::shutdown();
    return 0;
}
