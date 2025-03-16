// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class TurtleMover : public rclcpp::Node
{
public:
  TurtleMover()
  : Node("turtle_mover")
  {
    // Create a publisher for turtle velocity commands
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    // Lambda function to send movement commands every 3 seconds
    auto timer_callback =
      [this]() -> void {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 1.0;  // Move forward
        message.angular.z = 0.0; // No rotation

        RCLCPP_INFO(this->get_logger(), "Publishing movement command");
        this->publisher_->publish(message);
      };

    // Timer that triggers the callback every 3 seconds
    timer_ = this->create_wall_timer(3s, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleMover>());
  rclcpp::shutdown();
  return 0;
}
