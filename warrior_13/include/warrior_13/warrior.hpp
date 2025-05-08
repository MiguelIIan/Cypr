//--------------------------------------------------------------------//
//University of Málaga
//MAPIR Research Group - Machine Perception and Intelligent Robotics
//--------------------------------------------------------------------//
#include "rclcpp/rclcpp.hpp"
#include "jsoncpp/json/json.h"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rosgame_msgs/msg/rosgame_twist.hpp"
#include "rosgame_msgs/msg/rosgame_point.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rosgame_bridge/srv/rosgame_register.hpp"

class Warrior : public rclcpp::Node
{
public:
    Warrior();
    ~Warrior();

    // Función para procesar el sensor láser.
    void process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    // Función para procesar la información de la escena.
    void process_scene_info(const std_msgs::msg::String::SharedPtr msg);
    
    //puedes añadir mas funciones si lo crees oportuno

    void step();

    void forward();
    void turn(int s);

    int mas_cerca(std::vector<std::vector<float>> array);

    std::vector<float> point_composition(float x1,float y1,float gamma1,float x2,float y2);

    void go_to_point(std::vector<float> point, float v_gain, float w_gain);

    //PUBLICADORES
    //============
    // Publicador para enviar comandos de movimiento.
    rclcpp::Publisher<rosgame_msgs::msg::RosgameTwist>::SharedPtr pub1_;

    // Publicador para enviar comandos de posición.
    rclcpp::Publisher<rosgame_msgs::msg::RosgamePoint>::SharedPtr pub2_;

    //SUBSCRIPTORES
    //============
    // Subscripción al topic del sensor láser.
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
    
    // Subscripción al topic con la información de la escena.
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub2_;

    


private:
    // Variables de estado del robot. Puedes añadir las que necesites
    float battery;
    float pos_x, pos_y, gamma;
    bool autopilot_enabled = false, hammer_enabled = false, shield_enabled = false;
    
    //Id code for our robot. 
    std::string code = "-1";
    std::string warrior_nick ="";
    
    // Información de la escena.
    std::vector<std::vector<float>> skills_pos_array;
    std::vector<std::vector<float>> chargers_pos_array;
    std::vector<std::vector<float>> players_pos_array;

    // Variables auxiliares.
    // cualquier otra variable que puedas necesitar

    // Velocidad
    float v = 0.5;
    float w = 0.6;

    // Flags de estado de movimiento
    bool avanzando = true;
    bool girando_izq = false;
    bool girando_der = false;

    // Laser
    int n_ranges = 1;
    float nearest_obstacle_distance = 70;
    int pos_in_array = -1;

    float angle_min;
    float angle_increment;

    float dist_min = 1;
    int angle_limit = 70;

    // Skill/Escudo enemigo
    float epsilon = 0.4;
    std::vector<float> p_skill = {0, 0};
    std::vector<float> skill = {0,0};
    bool hay_skill = false;
    std::vector<float> p_player = {0, 0};
    std::vector<float> player = {0,0};
    bool hay_player = false;
};