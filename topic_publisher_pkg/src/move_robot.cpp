#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class CVPublisher : public rclcpp::Node {
public:
  CVPublisher() : Node("move_robot_circles") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&CVPublisher::mover_callback, this));
  }

private:
  void mover_callback() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.2;
    message.angular.z = 0.2;

    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CVPublisher>());
  rclcpp::shutdown();
  return 0;
}