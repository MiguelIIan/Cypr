//--------------------------------------------------------------------//
//University of Málaga
//MAPIR Research Group - Machine Perception and Intelligent Robotics
//--------------------------------------------------------------------//

#include "warrior_13/warrior.hpp"

Warrior::Warrior(): Node ("robot_warrior")
{
    warrior_nick = "warrior_13";
    int cont = 0;
    
    // Se crea un cliente de servicio y una solicitud para lanzar el servicio.
    auto client = create_client<rosgame_bridge::srv::RosgameRegister>("register_service");
    
    auto request = std::make_shared<rosgame_bridge::srv::RosgameRegister::Request>();
    request -> username = warrior_nick;
    
    // Se espera a que el servicio esté disponible.
    bool service_available = false;
    while(!service_available && rclcpp::ok())
    {
        if (client->wait_for_service(std::chrono::seconds(5)))
        {   service_available = true;   }
        else
        {   RCLCPP_INFO(this->get_logger(), "Service not available. Retrying...");  }
    }
    
    // Se llama al servicio hasta que la respuesta sea diferente a "-1". Este valor indica que ya existe un jugador registrado con el nombre de usuario proporcionado.
    while (code == "-1" && rclcpp::ok())
    {   
        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            code = response->code;

            if (code == "-1")
            {
                RCLCPP_WARN(this->get_logger(), "Username already exists. Calling the service again...");
                warrior_nick= warrior_nick + std::to_string(cont);
                request -> username = warrior_nick;
                cont = cont + 1;
            }
            else
            {   
                // Se definen los publicadores y suscriptores necesarios.
                pub1_ = create_publisher<rosgame_msgs::msg::RosgameTwist>( "/" + code + "/cmd_vel", 10 );
                pub2_ = create_publisher<rosgame_msgs::msg::RosgamePoint>( "/" + code + "/goal_x_y", 10 );
                sub1_ = create_subscription<sensor_msgs::msg::LaserScan>( "/" + code + "/laser_scan", 10, std::bind(&Warrior::process_laser_info, this, std::placeholders::_1));
                sub2_ = create_subscription<std_msgs::msg::String>( "/" + code + "/scene_info", 10, std::bind(&Warrior::process_scene_info, this, std::placeholders::_1));
                RCLCPP_INFO(this->get_logger(), "Player registered. Starting simulation.");           
            }
        }
    }




}

Warrior::~Warrior()
{
     RCLCPP_ERROR(this->get_logger(), "Game over for [%s]", warrior_nick.c_str());
}

void Warrior::process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    //Aquí tu código cada vez que recibas información del láser

    // PROCESAR DATOS

    // number of elements in array
    int n_ranges = msg->ranges.size();
    // search the minimum distance (closest object)
    std::vector<float>::iterator min_it = std::min_element(msg->ranges.begin(), msg->ranges.end());
    // get distance and position in array
    double nearest_obstacle_distance = *min_it;
    int pos_in_array = std::distance(msg->ranges.begin(), min_it);
    // inform
    RCLCPP_INFO(this->get_logger(), "Nearest obstacle detected at %.2f[m] at vector position %i/%i",
    nearest_obstacle_distance, pos_in_array, n_ranges);

    this->n_ranges = n_ranges;
    this->nearest_obstacle_distance = nearest_obstacle_distance;
    this->pos_in_array = pos_in_array;

    this->angle_min = msg->angle_min;
    this->angle_increment = msg->angle_increment;
}


void Warrior::process_scene_info(const std_msgs::msg::String::SharedPtr msg)
{
    // Se convierte el msg de tipo string con formato en un JSON.
    Json::CharReaderBuilder reader;
    Json::Value JsonSceneData;
    std::istringstream jsonStream(msg->data);
    Json::parseFromStream(reader, jsonStream, &JsonSceneData, nullptr);
        
    // Se obtiene el valor de la batería de la clave "Battery_Level".
    battery = JsonSceneData["Battery_Level"].asFloat();

    // Se obtiene la pose del robot a partir de la clave "Robot_Pose".
    pos_x = JsonSceneData["Robot_Pose"]["x"].asFloat();
    pos_y = JsonSceneData["Robot_Pose"]["y"].asFloat();
    gamma = JsonSceneData["Robot_Pose"]["gamma"].asFloat();

    // Se obtienen las habilidades de la cave "Skills".
    autopilot_enabled = JsonSceneData["Skills"]["Autopilot"].asBool();
    hammer_enabled = JsonSceneData["Skills"]["Hammer"].asBool();
    shield_enabled = JsonSceneData["Skills"]["Shield"].asBool();

    // Se obtienen las posiciones de bloques de habilidades de la clave "Skills_Positions" en el campo "FOV".
    std::vector<std::vector<float>> skills_pos_array_aux;
    const Json::Value &skills_pos = JsonSceneData["FOV"]["Skills_Positions"];
    for (const Json::Value &skill : skills_pos)
    {
        std::vector<float> skillData;
        for (const Json::Value &value : skill)
        {   skillData.push_back(value.asFloat());   }
        skills_pos_array_aux.push_back(skillData);
    }
    skills_pos_array = skills_pos_array_aux;

    // Se obtienen las posiciones de plataformas de recarga de la clave "Chargers_Positions" en el campo "FOV".
    // Las plataformas de recarga están en posiciones fijas durante toda la simulación.
    // Objetivo: buscar las cinco plataformas y almacenarlas todas en la lista "chargers_pos_array".
    const Json::Value &chargers_pos = JsonSceneData["FOV"]["Chargers_Positions"];
    for (const Json::Value &charger : chargers_pos)
    {
        std::vector<float> chargerData;
        for (const Json::Value &value : charger)
        {   chargerData.push_back(value.asFloat());     }

        // Verifica si la posición ya está en "chargers_pos_array".
        // La función "find" devuelve un iterador que apunta al elemento del array si existe o al final del array si no lo ha encontrado.
        if (std::find(chargers_pos_array.begin(), chargers_pos_array.end(), chargerData) == chargers_pos_array.end())
        {   chargers_pos_array.push_back(chargerData);  }
    }
    
    
    // Se obtienen las posiciones de los oponentes de la clave "Players_Positions" en el campo "FOV".
    std::vector<std::vector<float>> players_pos_array_aux;
    const Json::Value &players_pos = JsonSceneData["FOV"]["Players_Positions"];
    for (const Json::Value &player : players_pos)
    {
        std::vector<float> playerData;
        for (const Json::Value &value : player)
        {   playerData.push_back(value.asFloat());  }
        players_pos_array_aux.push_back(playerData);
    }
    players_pos_array = players_pos_array_aux;
    
    // DEBUGGING
    
    /*
    if (!chargers_pos_array.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Posicion de los cargadores en el array:");
        for (const auto &chargers : chargers_pos_array)
        {   RCLCPP_INFO(this->get_logger(), "[X] = '%f', [Y] = '%f'", chargers[0], chargers[1]);  }
    }*/

    /*
    RCLCPP_INFO(this->get_logger(), "Msg: '%s'", msg->data.c_str());

    RCLCPP_INFO(this->get_logger(), "Battery level: '%f'", battery);

    RCLCPP_INFO(this->get_logger(), "Is teleport enabled? '%s'", (teleport_enabled ? "True" : "False"));
    RCLCPP_INFO(this->get_logger(), "Is the hammer enabled? '%s'", (hammer_enabled ? "True" : "False"));
    RCLCPP_INFO(this->get_logger(), "Is the shield enabled? '%s'", (shield_enabled ? "True" : "False"));

    if (!skills_pos_array.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Skills Positions Array:");
        for (const auto &skill_pos : skills_pos_array)
        {   RCLCPP_INFO(this->get_logger(), "[X] = '%f', [Y] = '%f'", skill_pos[0], skill_pos[1]);  }
    }
    
    RCLCPP_INFO(this->get_logger(), "Robot Pose: [X] = '%f', [Y] = '%f', [GAMMA] = '%f'", pos_x, pos_y, gamma);
    */


}



// FUNCIONES DEFINIDAS PARA EL CONTROL

//---------FUNCIÓN EN EL QUE SE DECIDE QUE ACCIÓN TOMAR EN CADA MOMENTO----------------------------------------------------
void Warrior::step(){
    
    // DETECCIÓN A IZQUIERDA Y DERECHA CON LIMITACIÓN DE ÁNGULO
    float angulo_objeto_rad = (this->angle_min + (this->pos_in_array)*(this->angle_increment)); // Ángulo en grados
    float angulo_objeto = angulo_objeto_rad*180/(M_PI); // Ángulo en grados

    // RCLCPP_INFO(this->get_logger(), "ANGULO OBJETO: %.2f",angulo_objeto);

    
    // Calcular pose relativa objeto más cercano
    if(!skills_pos_array.empty()){
        skill = skills_pos_array[mas_cerca(skills_pos_array)];
        p_skill = this->point_composition(-(pos_x*cos(gamma)+pos_y*sin(gamma)),-(pos_y*cos(gamma)-pos_x*sin(gamma)),-gamma,skill[0],skill[1]);
        hay_skill = true;
    }else{
        hay_skill = false;
    }

    // Calcular pose relativa jugador más cercano
    if(!players_pos_array.empty()){
        player = players_pos_array[mas_cerca(players_pos_array)];
        p_player = this->point_composition(-(pos_x*cos(gamma)+pos_y*sin(gamma)),-(pos_y*cos(gamma)-pos_x*sin(gamma)),-gamma,player[0],player[1]);
        hay_player = true;
    }else{
        hay_player = false;
    }

    // Calcular pose relativa enemigo más cercano
    if(nearest_obstacle_distance <= dist_min){

        // Calcular pose relativa punto laser
        float x_laser = nearest_obstacle_distance*cos(angulo_objeto_rad);
        float y_laser = nearest_obstacle_distance*sin(angulo_objeto_rad);

        // Comparar pr_objeto == pr_laser? (epsilon > módulo diferencia vectores)
        float diff_skill = sqrt((x_laser-p_skill[0])*(x_laser-p_skill[0])+(y_laser-p_skill[1])*(y_laser-p_skill[1]));
        // Comparar pr_jugador == pr_laser? (epsilon > módulo diferencia vectores)
        float diff_jugador = sqrt((x_laser-p_player[0])*(x_laser-p_player[0])+(y_laser-p_player[1])*(y_laser-p_player[1]));

        //RCLCPP_INFO(this->get_logger(), "DIFF = '%f'", diff);
    
        if(diff_skill<epsilon && hay_skill){ // Objeto
            //RCLCPP_INFO(this->get_logger(), "Voy a por un objeto");
            this->go_to_point(p_skill,1.2,1);
        }else if( diff_jugador<epsilon && hay_player){// Escudo enemigo
            if(hammer_enabled){
                //RCLCPP_INFO(this->get_logger(), "Voy a por un enemigo");
                // Sistemas de referencia: Objeto = Robot*P_rel -> P_rel = Robot^(-1)*Objeto
                std::vector<float> p = this->point_composition(-(pos_x*cos(gamma)+pos_y*sin(gamma)),-(pos_y*cos(gamma)-pos_x*sin(gamma)),-gamma,player[0],player[1]);
                this->go_to_point(p,2,1.2);
            } else{
                //RCLCPP_INFO(this->get_logger(), "Estoy huyendo");
                p_player[0] = -p_player[0];
                p_player[1] = -p_player[1];
                this->go_to_point(p_player,2,1.6);
            }
        }else{ // Obstáculo
            //RCLCPP_INFO(this->get_logger(), "Estoy esquivando obstaculos");
            if((-angle_limit <= angulo_objeto) && (angulo_objeto < 0) && avanzando){ // Objeto derecha -> Giro izquierda
                avanzando = false;
                girando_izq = true;
            }else if (angulo_objeto >= 0 && angulo_objeto <angle_limit && avanzando){ // Objeto izquierda -> Giro derecha
                avanzando = false;
                girando_der = true;
            }else if(((-angle_limit >= angulo_objeto) || (angulo_objeto > angle_limit)) && !avanzando){
                // Sigue recto

                girando_izq = false;
                girando_der = false;
                avanzando = true;
                this->forward();
            }

            if(girando_izq){
                turn(1);
            }else if(girando_der){
                turn(-1);
            }
        }

    }else if(!players_pos_array.empty()){ // ATACAR
        if(hammer_enabled){
            //RCLCPP_INFO(this->get_logger(), "Voy a atacar");
            // Sistemas de referencia: Objeto = Robot*P_rel -> P_rel = Robot^(-1)*Objeto
            std::vector<float> p = this->point_composition(-(pos_x*cos(gamma)+pos_y*sin(gamma)),-(pos_y*cos(gamma)-pos_x*sin(gamma)),-gamma,player[0],player[1]);
            this->go_to_point(p,2,1.2);
        } else{
            //RCLCPP_INFO(this->get_logger(), "Estoy huyendo");
            p_player[0] = -p_player[0];
            p_player[1] = -p_player[1];
            this->go_to_point(p_player,2,1.6);
        }
    }else if(battery < 30 && !chargers_pos_array.empty()){ // elegir bien 
        //std::vector<float> p = {0, 0}; // Centro del mapa

        // Calcular cargador más cercano
        //RCLCPP_INFO(this->get_logger(), "Voy a recargarme");
        std::vector<float> cargador = chargers_pos_array[mas_cerca(chargers_pos_array)];
        std::vector<float> p = this->point_composition(-(pos_x*cos(gamma)+pos_y*sin(gamma)),-(pos_y*cos(gamma)-pos_x*sin(gamma)),-gamma,cargador[0],cargador[1]);
        this->go_to_point(p,1.2,1);
        
    }else if(!skills_pos_array.empty()){ // Objeto
        //RCLCPP_INFO(this->get_logger(), "Voy a por un objeto desde lejos");
        this->go_to_point(p_skill,1.2,1);
    }else{
        // Sigue recto
        //RCLCPP_INFO(this->get_logger(), "Solo avanzo");
        girando_izq = false;
        girando_der = false;
        avanzando = true;

        this->forward();
    }
}

//---------FUNCIÓN PARA PUBLICAR CUANDO QUEREMOS IR HACIA DELANTE------------------------------------------------
void Warrior::forward(){
    rosgame_msgs::msg::RosgameTwist cmd;

    cmd.vel.linear.x = this->v; 
    cmd.code = code;

    this->pub1_->publish(cmd);

}

//---------FUNCIÓN PARA GIRAR HACIA EN UN SENTIDO U OTRO SEGÚN LA ENTRADA--------------------------------------------
void Warrior::turn(int s){

    rosgame_msgs::msg::RosgameTwist cmd;

    cmd.vel.angular.z = s*(this->w);
    cmd.code = code;

    this->pub1_->publish(cmd);
}

//---------FUNCIÓN PARA OBTENER DE UNA POSICIÓN GLOBAL, LA RELATIVA AL ROBOT-------------------------------------------
std::vector<float> Warrior::point_composition(float x1,float y1,float gamma1,float x2,float y2){
    std::vector<float> p;

    p.push_back(x1 + x2*cos(gamma1) - y2*sin(gamma1));
    p.push_back(y1 + x2*sin(gamma1) + y2*cos(gamma1));
    
    return p;
}

//---------FUNCIÓN PARA IR A UN PUNTO GLOBAL CONCRETO(pose relativa al robot, situado en (0,0))------------------------------
void Warrior::go_to_point(std::vector<float> point, float v_gain, float w_gain){ // v_gain = 1 -> velocidad estándar
    rosgame_msgs::msg::RosgameTwist cmd;

    float x = point[0];
    float y = point[1];

    float angulo = atan2(y,x)*180/(M_PI);

    int angulo_1 = 75;
    int angulo_2 = 30;

    float g1 = 1.4;
    float g2 = 0.6;


    if(angulo > angulo_1){        // girar izquierda
        cmd.vel.angular.z = (this->w)*g1;
    }else if(angulo < -angulo_1){ // girar derecha
        cmd.vel.angular.z = -(this->w)*g1;
    }else if(angulo > angulo_2){
        cmd.vel.linear.x = (this->v);
        cmd.vel.angular.z = (this->w);
    }else if(angulo < -angulo_2){
        cmd.vel.linear.x = (this->v);
        cmd.vel.angular.z = -(this->w);
    }else if(angulo > 0){
        cmd.vel.linear.x = (this->v);
        cmd.vel.angular.z = (this->w)*g2;
    }else{ //if(angulo < 0){
        cmd.vel.linear.x = (this->v);
        cmd.vel.angular.z = -(this->w)*g2;
    }

    // Ganancias para las velocidades
    cmd.vel.linear.x = cmd.vel.linear.x*v_gain;
    cmd.vel.angular.z = cmd.vel.angular.z*w_gain;


    // PUBLICAR
    //RCLCPP_INFO(this->get_logger(), "POINT: '%f' '%f'", x, y);
    //RCLCPP_INFO(this->get_logger(), "CMD_VEL: '%f' '%f'", cmd.vel.linear.x, cmd.vel.angular.z);

    cmd.code = code;

    this->pub1_->publish(cmd);
}

//-----------FUNCIÓN PARA CALCULAR CUÁL DE LOS ELEMENTOS DEL ARRAY ESTÁ MÁS CERCA DE NUESTRO ROBOT-----------------------------
int Warrior::mas_cerca(std::vector<std::vector<float>> array){
    int indice_menor;
    // Solo hace el algoritmo de buscar el puesto de carga más cercano una vez
    float dist = 0;
    float dist1 = 20;
    int idx = 0;
    float x = 0;
    float y = 0;
    for (const auto &i : array){
        // Obtenemos los puntos de los cargadores relativos al robot, y luego calculamos la distancia para saber cuál es el más cercano
        std::vector<float> p = this->point_composition(-(pos_x*cos(gamma)+pos_y*sin(gamma)),-(pos_y*cos(gamma)-pos_x*sin(gamma)),-gamma,i[0],i[1]);
        x = p[0];
        y = p[1];
        dist = sqrt(pow(x,2) + pow(y,2));
        //RCLCPP_INFO(this->get_logger(),"La distancia a %i es: %f", idx,dist);
        // Nos vamos quedando en cada iteración con la menor distancia
        if ( dist<= dist1){
            dist1 = dist;
            indice_menor = idx;
        }
        idx++;
    }
    //RCLCPP_INFO(this->get_logger(),"La distancia mínima es del elemento: %i", indice_menor);
    //RCLCPP_INFO(this->get_logger(),"El número de cargadores es: %i", idx);
    
    // Devolvemos el indice en el que está guardado el punto más cercano
    return indice_menor;
}



