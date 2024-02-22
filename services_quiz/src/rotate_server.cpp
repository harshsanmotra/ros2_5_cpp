#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "services_quiz_srv/srv/spin.hpp"
#include <chrono>
#include <memory>
#include <string>

using SpinMessage = services_quiz_srv::srv::Spin;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

// Attention
// Service Call Proto //
// ros2 service call /rotate services_quiz_srv/srv/Spin "{direction: 'left',
// angular_velocity: 1.5, time: 10}"

class SpinServerNode : public rclcpp::Node {
public:
  SpinServerNode() : Node("spin_server") {

    srv_ = create_service<SpinMessage>(
        "rotate", std::bind(&SpinServerNode::rotate_callback, this, _1, _2));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<SpinMessage>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0;
    message.angular.z = 0;

    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "STOPPED");
  }

  void rotate_callback(const std::shared_ptr<SpinMessage::Request> request,
                       const std::shared_ptr<SpinMessage::Response> response) {

    auto message = geometry_msgs::msg::Twist();

    if (request->direction == "right" || request->direction == "left") {

      // Send velocities to move
      if (request->direction == "right")
        message.angular.z = -(request->angular_velocity);
      else
        message.angular.z = request->angular_velocity;
      publisher_->publish(message);

      int given_time = request->time;

      timer_ = this->create_wall_timer(
          std::chrono::seconds(given_time),
          std::bind(&SpinServerNode::timer_callback, this));

      // Set the response success variable to true
      response->success = true;
    } else {
      response->success = false;
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpinServerNode>());
  rclcpp::shutdown();
  return 0;
}