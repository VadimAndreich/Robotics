#include <functional>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define pi 3.14159265
#define angle pi / 4

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber() : Node("minimal_subscriber")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "cmd_text", 10, std::bind(&MinimalSubscriber::read_topic, this, _1));
  }

private:
  void read_topic(const std_msgs::msg::String & msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    auto linear_msg = linear_map_.find(msg.data);
    linear_speed_ = linear_msg != linear_map_.end() ? linear_msg -> second : 0;

    auto angular_msg = angular_map_.find(msg.data);
    angular_speed_ = angular_msg != angular_map_.end() ? angular_msg -> second : 0;
    
    publish();
  }

  void publish()
  {
    message_.linear.x = linear_speed_;
    message_.angular.z = angular_speed_;
    
    publisher_->publish(message_);
  }
  
  geometry_msgs::msg::Twist message_;
  double linear_speed_;
  double angular_speed_;

  std::map<std::string, double> linear_map_{{"move_forward", 1}, {"w", 1}, {"move_backward", -1}, {"s", -1}};
  std::map<std::string, double> angular_map_{{"turn_right", -1 * angle}, {"d", -1 * angle}, {"turn_left", angle}, {"a", angle}};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
