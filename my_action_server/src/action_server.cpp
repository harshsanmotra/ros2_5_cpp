#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "custom_interface/action/mover.hpp"

class MyActionServer : public rclcpp::Node {
public:
  /*Defines an alias Move for the custom action message type and GoalHandleMove
   * for handling goals of this action type.*/
  using Move = custom_interface::action::Mover;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

  /*Initializes the node with the name "my_action_server".
  Sets up the action server using rclcpp_action::create_server, specifying
  callbacks for handling goal requests, cancel requests, and accepted goals.
  Initializes a publisher to publish geometry_msgs::msg::Twist messages on the
  "cmd_vel" topic, which is commonly used in ROS to control robot movement.*/
  explicit MyActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("my_action_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Move>(
        this, "move_robot_as_2",
        std::bind(&MyActionServer::handle_goal, this, _1, _2),
        std::bind(&MyActionServer::handle_cancel, this, _1),
        std::bind(&MyActionServer::handle_accepted, this, _1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp_action::Server<Move>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  /*
  handle_goal: Called when a new goal is received. It logs the goal and
  returns ACCEPT_AND_EXECUTE, indicating that the goal should be accepted and
  executed.
  handle_cancel: Called when a request to cancel a goal is received.
  It logs the request and returns ACCEPT, indicating that the cancellation is
  accepted.
  handle_accepted: Called when a goal has been accepted. It starts a
  new thread to execute the goal, allowing the action server to quickly return
  and not block other operations.*/
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Move::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d",
                goal->secs);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}
        .detach();
  }
  /*This function is run in a separate thread for each accepted goal. It
  simulates moving the robot by publishing Twist messages to the "cmd_vel"
  topic. It checks for cancellation requests, publishes feedback during
  execution, and eventually sets the result of the action to indicate
  completion.*/
  void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Move::Feedback>();
    auto &message = feedback->feedback;
    message = "Starting movement...";
    auto result = std::make_shared<Move::Result>();
    auto move = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(1);

    for (int i = 0; (i < goal->secs) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->status = message;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Move robot forward and send feedback
      message = "Moving forward...";
      move.linear.x = 0.3;
      publisher_->publish(move);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->status = "Finished action server. Robot moved during 5 seconds";
      move.linear.x = 0.0;
      publisher_->publish(move);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
}; // class MyActionServer

/*
Initializes the ROS 2 system.
Creates an instance of MyActionServer.
Uses a MultiThreadedExecutor to handle callbacks and actions concurrently.
Spins the executor to process incoming requests and callbacks.
Shuts down ROS 2 before exiting the program.
*/
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<MyActionServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

/*
Okay. Try to analyze the code in more detail now that you have created your
first Action Server.

Begin with the first section, where you will import the necessary libraries.


#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "t3_action_msg/action/move.hpp"
Next, define your class MyActionServer that inherits from Node:


class MyActionServer : public rclcpp::Node
{
public:
  using Move = t3_action_msg::action::Move;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;
Note that you are also creating some "alias" to simplify the code.


explicit MyActionServer(const rclcpp::NodeOptions & options =
rclcpp::NodeOptions()) : Node("my_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Move>(
      this,
      "move_robot_as_2",
      std::bind(&MyActionServer::handle_goal, this, _1, _2),
      std::bind(&MyActionServer::handle_cancel, this, _1),
      std::bind(&MyActionServer::handle_accepted, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",
10);

  }
In the constructor of the class, initialize a ROS2 node named my_action_server.
Also, and important, create an ActionServer object to which you specify several
things:

The type of Action: Move.
The ROS2 node that contains the Action Server: in this case, this.
A callback method to be executed when the Action receives a goal: handle_goal.
A callback method to be executed if the Client cancels the current goal:
handle_cancel. A callback method to be executed if the goal is accepted:
handle_accepted. Finally, define a Publisher for the Topic /cmd_vel.

Now continue analyzing the handle_goal() method:


rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Move::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d",
goal->secs); (void)uuid; return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
This method will be called when the Action Server receives a goal. In this case,
you are accepting all the incoming goals with
rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE.

Next, find the handle_cancel() method:


rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
This method will be called when the Action Server receives a cancelation request
from the Client. In this case, you are accepting all the cancelation requests
with rclcpp_action::CancelResponse::ACCEPT.

Finally, find the handle_accepted() method:


void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
new thread std::thread{std::bind(&MyActionServer::execute, this, _1),
goal_handle}.detach();
  }
This method will be called when the Action Server accepts the goal. Here, you
are calling the execute function, which contains the main functionality of the
Action.

Finally, you have the execute method:


void execute(const std::shared_ptr<GoalHandleMove> goal_handle)
This method is the one that will do the real work. Review it step by step.
First, define all the variables that you will use:


const auto goal = goal_handle->get_goal();
auto feedback = std::make_shared<Move::Feedback>();
auto & message = feedback->feedback;
message = "Starting movement...";
auto result = std::make_shared<Move::Result>();
auto move = geometry_msgs::msg::Twist();
rclcpp::Rate loop_rate(1);
You have:

The goal variable, containing the goal message sent by the Client.

The feedback variable, containing the feedback message that you will send back
to the Client.

The result variable, containing the result message that you will send to the
Client when the Action finishes.

The move variable, containing the Twist message used to send velocities to the
robot.

A loop_rate variable of 1Hz (1 second).

Next, start a for loop that will keep executing until the variable i reaches the
number of seconds specified in the goal message.


for (int i = 0; (i < goal->secs) && rclcpp::ok(); ++i)
For instance, if you set five seconds in the goal message, this loop will be
executed once per second, for 5 seconds.

Next, check whether the Action has been canceled. If it is canceled, terminate
the Action.


if (goal_handle->is_canceling()) {
    result->status = message;
    goal_handle->canceled(result);
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    return;
  }
If the Action is not canceled, you will send a Twist message to the robot to
move it forward at a speed of 0.3 m/s, and you will send a feedback message back
to the Client with the string "Moving forward...".


message = "Moving forward...";
move.linear.x = 0.3;
publisher_->publish(move);
goal_handle->publish_feedback(feedback);
RCLCPP_INFO(this->get_logger(), "Publish feedback");
loop_rate.sleep();
At the end of the execute function, check whether the goal has been completed:


if (rclcpp::ok()) {
      result->status = "Finished action server. Robot moved during 5 seconds";
      move.linear.x = 0.0;
      publisher_->publish(move);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
If the goal has been accomplished, you will do two things:

Stop the robot by sending a velocity of 0.

Fill the result message and send it back to the Client.

Finally, find the main that looks like if we add to one MultiThreadedExecutor
inside it:


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<MyActionServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
As you can see, it is simple. Spin the Action Server node so that you keep the
Action Server up and running.

And that is it! Now everything makes more sense.
*/