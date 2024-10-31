#include <motion/motion_model_noise.hpp>
#include <random>
#include <motion/json.hpp>
using json = nlohmann::json;

const float Vp = 0.2;
const float w = M_PI/8;

MotionModel::MotionModel():Node("motion_model"){
    pub_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pub_pose = this->create_publisher<geometry_msgs::msg::Pose>("/estimated_pose", 10);
    pub_vector = this->create_publisher<std_msgs::msg::String>("/estimated_pose_array", 10);

    

    cont = 0;
    pose_actual.position.x = 0;
    pose_actual.position.y = 0;
    pose_actual.position.z = 0;
    pose_actual.orientation.x = 0;
    pose_actual.orientation.y = 0;
    pose_actual.orientation.z = 0;
    pose_actual.orientation.w = 0;

    estimated_pose_array.resize(100,pose_actual);
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
    geometry_msgs::msg::Pose poses_ruidosas;
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

    for (int i=0;i<=99;i++){
        pose_ruido = generate_noise_sample();
        p_inc_ruidosa = pose_composition(incremento_pose,pose_ruido);
        poses_ruidosas = pose_composition(estimated_pose_array[i],p_inc_ruidosa);
        estimated_pose_array[i] = poses_ruidosas;
    }

    pose_actual = pose_composition(pose_actual,incremento_pose);

    std_msgs::msg::String pose_json;
    pose_json.data = poses_to_json(estimated_pose_array);
    pub_vel -> publish(movimiento);
    pub_pose -> publish(pose_actual);
    pub_vector -> publish(pose_json);


    
}
geometry_msgs::msg::Pose MotionModel::generate_noise_sample(){
    // Seed with a real random value, if available
    std::random_device r;
    std::seed_seq seed2{r(), r(), r(), r(), r(), r(), r(), r()};
    std::mt19937 generator(seed2);
    // Noise Covariance_Matriz (only diagonal) [var_x, var_y, var_theta]
    std::vector<double> var_noise = {0.001, 0.001, 0.001};
    // normal_distribution(mean, std_dev)
    std::normal_distribution<> norm_dist_x(0,sqrt(var_noise[0]));
    std::normal_distribution<> norm_dist_y(0,sqrt(var_noise[1]));
    std::normal_distribution<> norm_dist_theta(0,sqrt(var_noise[2]));
    // Create a noise sample and store in a Pose variable
    geometry_msgs::msg::Pose noise_sample;
    noise_sample.position.x = norm_dist_x(generator);
    noise_sample.position.y = norm_dist_y(generator);
    noise_sample.orientation.z = norm_dist_theta(generator);
    return noise_sample;
}
std::string MotionModel::poses_to_json(std::vector<geometry_msgs::msg::Pose> vec_poses){
    json json_vector_poses; //creamos objeto tipo JSON (resultado)
    // Por cada pose de nuestro vector
    for (geometry_msgs::msg::Pose value : vec_poses)
    {
    // Transformamos a JSON
    json json_pose;
    json_pose["x"] = value.position.x;
    json_pose["y"] = value.position.y;
    json_pose["phi"] = value.orientation.z;
    // attach JSON value
    json_vector_poses.push_back(json_pose);
    }
    // return
    return json_vector_poses.dump(); // Esto es una cadena de texto!
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
