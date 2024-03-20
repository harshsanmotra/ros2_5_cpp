#include <inttypes.h>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_interface/action/mover.hpp"

/*Inherits from rclcpp::Node, making it a specialized ROS 2 node capable of
 * communication over ROS topics, services, and actions.*/
class MyActionClient : public rclcpp::Node {
public:
  /*using Directives: Defines aliases for the Move action and its goal handle to
   * simplify code.*/
  using Move = custom_interface::action::Mover;
  using GoalHandleMove = rclcpp_action::ClientGoalHandle<Move>;

  /*Initializes the node with a name ("my_action_client") and sets up the action
   * client for the "move_robot_as" action server. A timer is created to
   * periodically call send_goal().*/

  explicit MyActionClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("my_action_client", node_options), goal_done_(false) {
    this->client_ptr_ = rclcpp_action::create_client<Move>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "move_robot_as_2");

    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&MyActionClient::send_goal, this));
  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    /*Cancels any existing timers (to avoid sending multiple goals), checks if
     * the action client is properly initialized and if the action server is
     * available, then sends a goal to move the robot for 5 seconds.
     */
    using namespace std::placeholders;

    this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = Move::Goal();
    goal_msg.secs = 5;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Move>::SendGoalOptions();

    /*goal_response_callback(): Called when the action server responds to the
     * goal. It checks if the goal was accepted.*/
    send_goal_options.goal_response_callback =
        std::bind(&MyActionClient::goal_response_callback, this, _1);
    /*feedback_callback(): Receives feedback from the action server while the
     * goal is being processed.*/
    send_goal_options.feedback_callback =
        std::bind(&MyActionClient::feedback_callback, this, _1, _2);
    /*result_callback(): Called when the action is completed. It checks the
     * result and prints a message based on whether the action succeeded, was
     * aborted, or canceled*/
    send_goal_options.result_callback =
        std::bind(&MyActionClient::result_callback, this, _1);

    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  /*
  client_ptr_: A shared pointer to the action client.
  timer_: A shared pointer to a timer object used to control the timing of goal
  sending. goal_done_: A boolean flag indicating whether the goal has been
  achieved.*/
  rclcpp_action::Client<Move>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

  void goal_response_callback(const GoalHandleMove::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleMove::SharedPtr,
                         const std::shared_ptr<const Move::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Feedback received: %s",
                feedback->feedback.c_str());
  }

  void result_callback(const GoalHandleMove::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received: %s",
                result.result->status.c_str());
  }
}; // class MyActionClient

/*
+----------------+        +-------------------+       +-----------------+
|                |        |                   |       |                 |
| MyActionClient +------->+ rclcpp_action     +------>+ Action Server   |
|                |  send  | (Action Client)   |  goal | (move_robot_as) |
+------+---------+        +-------------------+       +-------+---------+
       ^                                                        |
       |                                                        |
       |                     +------------------+              |
       +---------------------+ rclcpp::Node     |<-------------+
                             | (Base Class)     |
                             +------------------+

*/

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<MyActionClient>();
  /*Runs a MultiThreadedExecutor to keep the node alive and process callbacks
   * until the goal is done.*/
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_client);

  while (!action_client->is_goal_done()) {
    executor.spin();
  }

  rclcpp::shutdown();
  return 0;
}