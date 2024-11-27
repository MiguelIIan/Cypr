#include <navigation/reactive.hpp>
#include "math.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
const float Vp = 0.1;
const float w = 0.5;


Reactivo::Reactivo():Node("reactive"){
    pub_vel = this->create_publisher<geometry_msgs::msg::Twist>("/PioneerP3DX/cmd_vel", 10);

    sub_laser = this->create_subscription<sensor_msgs::msg::LaserScan>("/PioneerP3DX/laser_scan",10,std::bind(&Reactivo::process_laser,this,_1));
    sub_pose = this->create_subscription<geometry_msgs::msg::Pose>("/PioneerP3DX/ground_truth",10,std::bind(&Reactivo::estimated_pose,this,_1));
    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("/PioneerP3DX/odom",10,std::bind(&Reactivo::odom,this,_1));

    server_status = this->create_service<navigation::srv::Status>("servicio_estado",std::bind(&Reactivo::handle_status_service,this,_1,_2,_3));


    giroder = 0;
    giroizq = 0;
    pasos = 0;

    movimiento.linear.x = 0;
    movimiento.linear.y = 0;
    movimiento.angular.z = 0;

    derecha = false;
    izquierda = false;
    inicio = true;
}

Reactivo::~Reactivo(){
    printf("Cerrando proceso...\n");
}

void Reactivo::proximo_paso(){
    
    if (nearest_obstacle_distance < 0.4){
        if ((pos_in_array>= 342 && pos_in_array<=520 && izquierda==false)|| derecha == true){
            if (derecha != true){
                giroder++;
            }
            derecha = true;
            movimiento.linear.x = 0;
            movimiento.linear.y =0;
            movimiento.angular.z = -w;
        } else if ((pos_in_array < 342 && pos_in_array>160 && derecha==false) || izquierda == true){
            if (izquierda != true){
                giroizq++;
            }
            izquierda = true;
            movimiento.linear.x = 0;
            movimiento.linear.y =0;
            movimiento.angular.z = w;
        } 
        if ((pos_in_array > 0 && pos_in_array<160) || (pos_in_array < 684 && pos_in_array>520)){
            derecha = false;
            izquierda = false;
            movimiento.linear.x = Vp;
            movimiento.linear.y =0;
            movimiento.angular.z = 0;
        }
    } else {
        derecha = false;
        izquierda = false;
        movimiento.linear.x = Vp;
        movimiento.linear.y =0;
        movimiento.angular.z = 0;
    }
    
    pub_vel -> publish(movimiento);
}

void Reactivo::estimated_pose(const geometry_msgs::msg::Pose::SharedPtr groundtruth){
    double dist = 0;

    if (inicio == true){
        inicio = false;
        posicion_ant.position.x = groundtruth->position.x;
        posicion_ant.position.y = groundtruth->position.y;
    }

    dist = sqrt(pow(groundtruth->position.x - posicion_ant.position.x,2) + pow(groundtruth->position.y - posicion_ant.position.y,2));
    
    pasos = pasos + dist;

    RCLCPP_INFO(this->get_logger(),"Valor en x del groundtruth: %f", groundtruth->position.x);
    RCLCPP_INFO(this->get_logger(),"Valor en y del groundtruth: %f", groundtruth->position.y);
    RCLCPP_INFO(this->get_logger(),"Distancia total: %f", pasos);

    posicion_ant.position.x = groundtruth->position.x;
    posicion_ant.position.y = groundtruth->position.y;
}

void Reactivo::odom(const nav_msgs::msg::Odometry::SharedPtr odometria){
    //RCLCPP_INFO(this->get_logger(),"Valor de la odometría: %f", odometria->twist.twist.linear.x);
}

void Reactivo::process_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    // number of elements in array
    int n_ranges = msg->ranges.size();
    // search the minimum distance (closest object)
    std::vector<float>::iterator min_it = std::min_element(msg->ranges.begin(), msg->ranges.end());
    // get distance and position in array
    nearest_obstacle_distance = *min_it;
    pos_in_array = std::distance(msg->ranges.begin(), min_it);
    // inform
    //RCLCPP_INFO(this->get_logger(), "Nearest obstacle detected at %.2f[m] at vector position %i/%i",nearest_obstacle_distance, pos_in_array, n_ranges);
}

void Reactivo::handle_status_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navigation::srv::Status::Request> request,
    std::shared_ptr<navigation::srv::Status::Response> response)
{
    RCLCPP_INFO(this->get_logger(),"Has llamado al servicio de estado");

    response->giroderecha=giroder;
    response->giroizquierda=giroizq;
    response->distancia=pasos;

}

//---------------------
//       Main
//---------------------
int main( int argc, char * argv[] )
{
    // init ROS2 node
    rclcpp::init( argc, argv );
    // Create object from our MotionModel class
    auto node = std::make_shared<Reactivo>();
    // Running at 10Hz
    rclcpp::Rate loop_rate(10.0);
    RCLCPP_INFO(node->get_logger(), "Starting Main Loop...");
    while (rclcpp::ok()){
        rclcpp::spin_some(node); // attend subscriptions and srv request
        node->proximo_paso(); // send twist to robot and update pose estimation
        loop_rate.sleep(); // sleep till next step time
    }
    rclcpp::shutdown();
    return 0;
}