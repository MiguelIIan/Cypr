#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "services/srv/clear.hpp"

class Server: public rclcpp::Node{
    public:
        Server();
        ~Server();
    private:
        int cont_;
        int peticiones_;
        rclcpp::Service<services::srv::Clear>::SharedPtr server_start;
        void handle_start_service(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<services::srv::Clear::Request> request,
            std::shared_ptr<services::srv::Clear::Response> response
        )
};