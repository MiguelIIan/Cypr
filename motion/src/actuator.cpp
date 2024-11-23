#include <motion/actuator.hpp>

using std::placeholders::_1;

Actuator::Actuator():Node("actuator"){
    pub = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);
    sub = this->create_subscription<std_msgs::msg::String>("/command",10,std::bind(&Actuator::execute_command,this,_1));
}
Actuator::~Actuator(){
    printf("Saliendo del proceso.\n");
}
void Actuator::execute_command(const std_msgs::msg::String::SharedPtr msg){
    geometry_msgs::msg::Twist actuation;
    float angular, linear;
    RCLCPP_INFO_STREAM(this->get_logger(),"Recibido #: "<<msg->data);
    if (msg->data == "stop"){linear = 0; angular = 0;}
    else if (msg->data == "adelante"){linear = 1; angular = 0;}
    else if (msg->data == "atras"){linear = -1; angular = 0;}
    else if (msg->data == "derecha"){linear = 0; angular = 1;}
    else if (msg->data == "izquierda"){linear = 0; angular = -1;}
    actuation.linear.x = linear;
    actuation.angular.z = angular;

    pub -> publish(actuation);
}
