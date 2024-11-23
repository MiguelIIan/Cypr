#include <motion/controller.hpp>

Controller::Controller():Node("controller"){
    pub = this->create_publisher<std_msgs::msg::String>("/command",10);
    cont = 0; 
}
Controller::~Controller(){
    printf("Saliendo del proceso.\n");
}
void Controller::publish_method(){
    std_msgs::msg::String mensaje;
    int random = round(4*(float)rand()/(float)RAND_MAX);
    cont++;
    switch(random){
        case 0: mensaje.data = "stop";break;
        case 1: mensaje.data = "adelante";break;
        case 2: mensaje.data = "atras";break;
        case 3: mensaje.data = "derecha";break;
        case 4: mensaje.data = "izquierda";break;
    }
    RCLCPP_INFO_STREAM(this->get_logger(),"Publishing #: "<<cont<<" - "<<mensaje.data);
    pub -> publish(mensaje);
}