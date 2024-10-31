#include <motion/motion_model.hpp>

const float Vp = 0.2;
const float w = M_PI/8;

MotionModel::MotionModel():Node("motion_model"){
    pub_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pub_pose = this->create_publisher<geometry_msgs::msg::Pose>("/estimated_pose", 10);

    cont = 0;
    pose_actual.position.x = 0;
    pose_actual.position.y = 0;
    pose_actual.position.z = 0;
    pose_actual.orientation.x = 0;
    pose_actual.orientation.y = 0;
    pose_actual.orientation.z = 0;
    pose_actual.orientation.w = 0;
}
MotionModel::~MotionModel(){
    printf("Terminando el proceso");
}
geometry_msgs::msg::Pose MotionModel::pose_composition(geometry_msgs::msg::Pose p1,geometry_msgs::msg::Pose p2){
    geometry_msgs::msg::Pose pose_actualizada;
    pose_actualizada.position.x = p1.position.x + p2.position.x*cos(p1.orientation.z) - p2.position.y*sin(p1.orientation.z);
    pose_actualizada.position.y = p1.position.y + p2.position.x*sin(p1.orientation.z) + p2.position.y*cos(p1.orientation.z);
    pose_actualizada.orientation.z = p1.orientation.z + p2.orientation.z;
    return pose_actualizada;
}
void MotionModel::step_forward(){
    geometry_msgs::msg::Pose incremento_pose;
    geometry_msgs::msg::Twist movimiento;
    
    cont++;
    if (cont >= 76){
        
        incremento_pose.position.x = 0;                        //  (Vp/w)*sin(w)
        incremento_pose.position.y = 0;                      //(Vp/w)*(1 - cos(w));
        incremento_pose.orientation.z = -w*0.2;

        movimiento.linear.x = 0;
        movimiento.linear.y =0;
        movimiento.angular.z = -w;

        if (cont >= 95){
            cont = 0;
        }
    } else{
        incremento_pose.position.x = Vp*0.2;
        incremento_pose.position.y = 0;
        incremento_pose.orientation.z = 0;
        
        movimiento.linear.x = Vp;
        movimiento.linear.y =0;
        movimiento.angular.z = 0;
    }
    pose_actual = pose_composition(pose_actual,incremento_pose);
    pub_vel -> publish(movimiento);

    pub_pose -> publish(pose_actual);
    
}



int main( int argc, char * argv[] )
{
    // init ROS2 node
    rclcpp::init( argc, argv );
    // Create object from our MotionModel class
    auto node = std::make_shared<MotionModel>();
    // Running at 5Hz
    rclcpp::Rate loop_rate(5.0);
    RCLCPP_INFO(node->get_logger(), "Starting Main Loop...");
    while (rclcpp::ok()){
        rclcpp::spin_some(node); // attend subscriptions and srv request
        node->step_forward(); // send twist to robot and update pose estimation
        loop_rate.sleep(); // sleep till next step time
    }
    rclcpp::shutdown();
    return 0;
}