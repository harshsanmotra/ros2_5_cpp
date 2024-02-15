#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

class AvoidObstacle : public rclcpp::Node {
public:
  AvoidObstacle() : Node("avoid_obs") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 2);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&AvoidObstacle::decide_callback, this,
                  std::placeholders::_1));
  }

  float
  get_front_laser_reading(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // code to get laser reading in front of the robot
    return msg->ranges[msg->ranges.size() / 2];
  }

  float
  get_right_laser_reading(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // code to get laser reading on the right side of the robot
    return msg->ranges[0];
  }

  float get_mid_right_laser_reading(
      const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // code to get laser reading on the right side of the robot
    return msg->ranges[msg->ranges.size() / 4];
  }

  float
  get_left_laser_reading(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // code to get laser reading on the left side of the robot
    return msg->ranges[msg->ranges.size() - 1]; // fixed index
  }

  float
  get_mid_left_laser_reading(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // code to get laser reading on the left side of the robot
    return msg->ranges[msg->ranges.size() * (3 / 4)];
  }

  void move_forward(geometry_msgs::msg::Twist &msg) {
    // code to make the robot move forward
    msg.linear.x = 0.5;
    msg.angular.z = 0.0;
  }

  void turn_left(geometry_msgs::msg::Twist &msg) {
    // code to make the robot turn left
    msg.linear.x = 0.0;
    msg.angular.z = 0.5; // reduce the turning speed
  }

  void turn_right(geometry_msgs::msg::Twist &msg) {
    // code to make the robot turn right
    msg.linear.x = 0.0;
    msg.angular.z = -0.5; // reduce the turning speed
  }

  void move_stop(geometry_msgs::msg::Twist &msg) {
    // code to make the robot stop
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
  }

private:
  void decide_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    geometry_msgs::msg::Twist cmd_vel_msg;

    float front_reading = get_front_laser_reading(msg);
    float right_reading = get_right_laser_reading(msg);
    float left_reading = get_left_laser_reading(msg);
    float mid_right_reading = get_mid_right_laser_reading(msg);
    float mid_left_reading = get_mid_left_laser_reading(msg);

    std::cout << "Front Reading: " << front_reading << std::endl;
    std::cout << "Right Reading: " << right_reading << std::endl;
    std::cout << "Left Reading: " << left_reading << std::endl;

    // Threshold values for obstacle detection
    float obstacle_threshold = 1.0; // Distance to consider an obstacle
    float turn_threshold = 0.5;     // Distance for sharp turns

    if (front_reading > obstacle_threshold) {
      // Clear path ahead
      move_forward(cmd_vel_msg);
    } else {
      // Obstacle detected
      if (right_reading > obstacle_threshold &&
          mid_right_reading > obstacle_threshold) {
        // Obstacle on the right, sharp turn to the left
        turn_left(cmd_vel_msg);
      } else if (left_reading > obstacle_threshold &&
                 mid_left_reading > obstacle_threshold) {
        // Obstacle on the left, sharp turn to the right
        turn_right(cmd_vel_msg);
      } else {
        // Obstacles on both sides or very close, reverse and turn
        move_stop(cmd_vel_msg);
        if (right_reading < turn_threshold && left_reading < turn_threshold) {
          // If both sides are equally close, choose a random direction
          if (rand() % 2 == 0) {
            turn_left(cmd_vel_msg);
          } else {
            turn_right(cmd_vel_msg);
          }
        } else if (right_reading < turn_threshold) {
          // Obstacle on the right, turn left
          turn_left(cmd_vel_msg);
        } else if (left_reading < turn_threshold) {
          // Obstacle on the left, turn right
          turn_right(cmd_vel_msg);
        } else {
          // If obstacle detected but not too close on sides, turn left by
          // default
          turn_left(cmd_vel_msg);
        }
      }
    }

    publisher_->publish(cmd_vel_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AvoidObstacle>());
  rclcpp::shutdown();
  return 0;
}
