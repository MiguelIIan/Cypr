#include <control_brazo/arm_controller.hpp>

using std::placeholders::_1;


ArmController::ArmController():Node("arm_controller"){
    pub_vel = this->create_publisher<std_msgs::msg::String>("cmd_arm", 10);
    pub_goto = this->create_publisher<geometry_msgs::msg::Pose>("/goto",10);

    sub_pose = this->create_subscription<geometry_msgs::msg::Pose>("/currentPose",10,std::bind(&ArmController::target_pose,this,_1));
    sub_laser = this->create_subscription<std_msgs::msg::Float32>("/laser_data",10,std::bind(&ArmController::procesa_laser,this,_1));

    // Inicializamos las variables
    teclado = true;
    x = 0.0;
    y = 0.0;
    z = 0.0;
}
ArmController::~ArmController(){
    RCLCPP_INFO(this->get_logger(), "Cerrando Controlador del brazo");
}
void ArmController::procesa_laser(const std_msgs::msg::Float32::SharedPtr Laser){
    //No hacemos nada con la información del laser
}
void ArmController::target_pose(const geometry_msgs::msg::Pose::SharedPtr TargetPose){
    //Guardamos en la variable "punto" cada una de las poses del brazo
    punto.position.x = TargetPose->position.x;
    punto.position.y = TargetPose->position.y;
    punto.position.z = TargetPose->position.z;
    punto.orientation.x = TargetPose->orientation.x;
    punto.orientation.y = TargetPose->orientation.y;
    punto.orientation.z = TargetPose->orientation.z;
    punto.orientation.w = TargetPose->orientation.w;

}
void ArmController::orden_teclado(){
    
    //Creamos la variable en la que guardaremos los mensajes de nuestro nodo para enviarlo
    std_msgs::msg::String mensaje;
    if (teclado == true){
        system("stty raw");
        // Leemos un caracter
        char input = getchar();
        switch (input)
        {
            case 'q':
                RCLCPP_INFO(this->get_logger(), "Avanzando en X\r\n");
                mensaje.data = "X+";
                pub_vel -> publish(mensaje);
                break;
            case 'w':
                RCLCPP_INFO(this->get_logger(), "Retrocediendo en X\r\n");
                mensaje.data = "X-";
                pub_vel -> publish(mensaje);
                break;
            case 'a':
                RCLCPP_INFO(this->get_logger(), "Avanzando en Y\r\n");
                mensaje.data = "Y+";
                pub_vel -> publish(mensaje);
                break;
            case 's':
                RCLCPP_INFO(this->get_logger(), "Retrocediendo en Y\r\n");
                mensaje.data = "Y-";
                pub_vel -> publish(mensaje);
                break;
            case 'z':
                RCLCPP_INFO(this->get_logger(), "Avanzando en Z\r\n");
                mensaje.data = "Z+";
                pub_vel -> publish(mensaje);
                break;
            case 'x':
                RCLCPP_INFO(this->get_logger(), "Retrocediendo en Z\r\n");
                mensaje.data = "Z-";
                pub_vel -> publish(mensaje);
                break;
            case 'h':
                RCLCPP_INFO(this->get_logger(), "Moviendose a HOME\r\n");
                mensaje.data = "home";
                pub_vel -> publish(mensaje);
                break;
            case 'p':
                //Guardamos en nuestra variable "target" la pose actual
                target.position.x = punto.position.x;
                target.position.y = punto.position.y;
                target.position.z = punto.position.z;
                target.orientation.x = punto.orientation.x;
                target.orientation.y = punto.orientation.y;
                target.orientation.z = punto.orientation.z;
                target.orientation.w = punto.orientation.w;
                RCLCPP_INFO(this->get_logger(),"\r\nPose guardada: \r\n "
                                                "Posicion: \r\n "
                                                "X: %f\r\n Y: %f\r\n Z: %f\r\n "
                                                "Orientacion: \r\n "
                                                "X: %f\r\n Y: %f\r\n Z: %f\r\n W: %f\r\n",
                                                target.position.x,target.position.y,target.position.z,
                                                target.orientation.x,target.orientation.y,target.orientation.z,target.orientation.w);
                break;
            case 'g':
                RCLCPP_INFO(this->get_logger(), "Yendo a la Pose guardada\r\n");
                pub_goto -> publish(target);
                break;
            case 0x20:
                RCLCPP_INFO(this->get_logger(), "Detectado ESPACIO ====> SALIENDO");
                teclado = false;
                //Recuperamos el control de la consola
                system("stty cooked");
                break;
            default:
                // ignoramos caracteres no deseados
                break; 
        }
    }
    
}



//---------------------
//       Main
//---------------------
int main( int argc, char * argv[] )
{
    // init ROS2 node
    rclcpp::init( argc, argv );
    // Create object from our MotionModel class
    auto node = std::make_shared<ArmController>();
    // Running at 10Hz
    rclcpp::Rate loop_rate(10.0);
    RCLCPP_INFO(node->get_logger(), "Empezando bucle inicial...");
    while (rclcpp::ok()){
        rclcpp::spin_some(node); // attend subscriptions and srv request
        node->orden_teclado(); // send twist to robot and update pose estimation
        loop_rate.sleep(); // sleep till next step time
    }
    rclcpp::shutdown();
    return 0;
}