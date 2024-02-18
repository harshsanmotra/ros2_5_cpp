#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <memory>

using _SetBool = std_srvs::srv::SetBool;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("service_turn_or_stop") {

    srv_ = create_service<_SetBool>(
        "turn_or_stop",
        std::bind(&ServerNode::turn_or_stop_callback, this, _1, _2));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<_SetBool>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void
  turn_or_stop_callback(const std::shared_ptr<_SetBool::Request> request,
                        const std::shared_ptr<_SetBool::Response> response) {

    auto message = geometry_msgs::msg::Twist();

    if (request->data == true) {

      RCLCPP_INFO(this->get_logger(), "Something right happening...");
      message.linear.x = 0.2;
      message.angular.z = -0.2;
      publisher_->publish(message);
      //    --- Setting the response success and message
      response->success = true;
      response->message = "Turning to the right right right!";
    }
    if (request->data == false) {

      RCLCPP_INFO(this->get_logger(), "Something false happening...");
      message.linear.x = 0.0;
      message.angular.z = 0.0;
      publisher_->publish(message);
      //    --- Setting the response success and message
      response->success = false;
      response->message = "It is time to stop.";
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}