#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "services/srv/clear.hpp"
#include "services/srv/set.hpp"
#include "services/srv/get.hpp"

class Server: public rclcpp::Node{
    public:
        Server();
        ~Server();
        void cuenta();
    private:
        int cont_;
        int peticiones_;
        rclcpp::Service<services::srv::Clear>::SharedPtr server_clear;
        void clear_service(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<services::srv::Clear::Request> request,
            std::shared_ptr<services::srv::Clear::Response> response
        );
        rclcpp::Service<services::srv::Set>::SharedPtr server_set;
        void set_service(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<services::srv::Set::Request> request,
            std::shared_ptr<services::srv::Set::Response> response
        );
        rclcpp::Service<services::srv::Get>::SharedPtr server_get;
        void get_service(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<services::srv::Get::Request> request,
            std::shared_ptr<services::srv::Get::Response> response
        );
};