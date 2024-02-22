#include "rclcpp/rclcpp.hpp"
#include "services_quiz_srv/srv/spin.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>

using namespace std::chrono_literals;
using SpinMessage = services_quiz_srv::srv::Spin;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("rotate_client");
  rclcpp::Client<SpinMessage>::SharedPtr client =
      node->create_client<SpinMessage>("rotate");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Service not available, waiting again...");
  }

  auto request = std::make_shared<SpinMessage::Request>();
  request->direction = "right";
  request->angular_velocity = 0.2;
  request->time = 10;

  auto result_future = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->success == true) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned success");

      // Create a timer to wait for the specified time before shutting down
      rclcpp::TimerBase::SharedPtr timer =
          node->create_wall_timer(std::chrono::seconds(request->time), [&]() {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                        "Rotation finished. Shutting down...");
            rclcpp::shutdown();
          });
    } else if (result->success == false) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned false");
      rclcpp::shutdown();
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service /moving");
    rclcpp::shutdown();
  }

  rclcpp::spin(node); // Spin the node until the timer triggers
  return 0;
}
