#include <services/servernode.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

Server::Server() : Node ("ServerNode"){
    server_clear = this->create_service<services::srv::Clear>("servicio_clear", std::bind(&Server::clear_service,this,_1,_2,_3));
    
    server_set = this->create_service<services::srv::Set>("servicio_set", std::bind(&Server::set_service,this,_1,_2,_3));

    server_get = this->create_service<services::srv::Get>("servicio_get", std::bind(&Server::get_service,this,_1,_2,_3));

    cont_ = 0;
}
Server::~Server(){
    printf("Cerrando Servidor.");
}

void Server::cuenta(){
    cont_++;
}


void Server::clear_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<services::srv::Clear::Request> request,
    std::shared_ptr<services::srv::Clear::Response> response)
{
    RCLCPP_INFO(this->get_logger(),"Reinicio del contador");
    cont_ = 0;
    peticiones_++;
}
void Server::get_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<services::srv::Get::Request> request,
    std::shared_ptr<services::srv::Get::Response> response)
{
    RCLCPP_INFO(this->get_logger(),"El contador vale: %i", cont_);
    response->valor = cont_;
    peticiones_++;
}
void Server::set_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<services::srv::Set::Request> request,
    std::shared_ptr<services::srv::Set::Response> response)
{
    RCLCPP_INFO(this->get_logger(),"Asignación del contador");
    cont_ = request->contador;
    response->peticiones = peticiones_ + 1;
    peticiones_=0;
}

int main(int argc,char* argv[]){
    rclcpp::init (argc,argv);
    auto node=std::make_shared<Server>();

    rclcpp::Rate loop_rate(1);
    while (rclcpp::ok()){
        rclcpp::spin_some(node);
        node->cuenta();
        loop_rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}