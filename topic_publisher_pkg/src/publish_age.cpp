#include "custom_interface/msg/age.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdio>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class AgePublisher : public rclcpp::Node {
public:
  AgePublisher() : Node("age_publisher") {
    publisher_ =
        this->create_publisher<custom_interface::msg::Age>("age", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&AgePublisher::age_callback, this));
  }

private:
  void age_callback() {
    auto message = custom_interface::msg::Age();
    message.years = 4;
    message.months = 5;
    message.days = 26;

    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_interface::msg::Age>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgePublisher>());
  rclcpp::shutdown();
  return 0;
}