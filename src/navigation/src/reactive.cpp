#include <navigation/reactive.hpp>

using std::placeholders::_1;

const float Vp = 0.1;
const float w = 0.5;


Reactivo::Reactivo():Node("reactive"){
    pub_vel = this->create_publisher<geometry_msgs::msg::Twist>("/PioneerP3DX/cmd_vel", 10);

    sub_laser = this->create_subscription<sensor_msgs::msg::LaserScan>("/PioneerP3DX/laser_scan",10,std::bind(&Reactivo::process_laser,this,_1));
    sub_pose = this->create_subscription<geometry_msgs::msg::Pose>("/PioneerP3DX/ground_truth",10,std::bind(&Reactivo::estimated_pose,this,_1));
    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("/PioneerP3DX/odom",10,std::bind(&Reactivo::odom,this,_1));

    movimiento.linear.x = 0;
    movimiento.linear.y = 0;
    movimiento.angular.z = 0;
}

Reactivo::~Reactivo(){
    printf("Cerrando proceso...\n");
}

void Reactivo::proximo_paso(){
    bool derecha = false;
    bool izquierda = false;

    if (nearest_obstacle_distance < 0.5){
        if (pos_in_array>= 360 && pos_in_array<=500 && derecha==false){
            izquierda = true;
            movimiento.linear.x = 0;
            movimiento.linear.y =0;
            movimiento.angular.z = -w;
        } else if (pos_in_array < 324 && pos_in_array>180 && izquierda==false){
            derecha = true;
            movimiento.linear.x = 0;
            movimiento.linear.y =0;
            movimiento.angular.z = w;
        } else if (pos_in_array > 324 && pos_in_array<360 || pos_in_array < 324 && pos_in_array>180 && pos_in_array> 360 && pos_in_array<500){
            movimiento.linear.x = -Vp;
            movimiento.linear.y =0;
            movimiento.angular.z = 2*w;
        } else {
            derecha = false;
            izquierda = false;
            movimiento.linear.x = Vp;
            movimiento.linear.y =0;
            movimiento.angular.z = 0;
        }
    } else {
        movimiento.linear.x = Vp;
        movimiento.linear.y =0;
        movimiento.angular.z = 0;
    }
    
    pub_vel -> publish(movimiento);
}

void Reactivo::estimated_pose(const geometry_msgs::msg::Pose::SharedPtr groundtruth){
    printf("hola");
}

void Reactivo::odom(const nav_msgs::msg::Odometry::SharedPtr odometria){
    printf("adios");
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
    RCLCPP_INFO(this->get_logger(), "Nearest obstacle detected at %.2f[m] at vector position %i/%i",
        nearest_obstacle_distance, pos_in_array, n_ranges);
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
    // Running at 5Hz
    rclcpp::Rate loop_rate(5.0);
    RCLCPP_INFO(node->get_logger(), "Starting Main Loop...");
    while (rclcpp::ok()){
        rclcpp::spin_some(node); // attend subscriptions and srv request
        node->proximo_paso(); // send twist to robot and update pose estimation
        loop_rate.sleep(); // sleep till next step time
    }
    rclcpp::shutdown();
    return 0;
}