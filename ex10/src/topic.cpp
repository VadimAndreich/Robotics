#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher() : Node("minimal_publisher"), stop_(true)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("cmd_text", 10);

    commands_.push_back("turn_right");
    commands_.push_back("d");
    commands_.push_back("turn_left");
    commands_.push_back("a");
    commands_.push_back("move_forward"); 
    commands_.push_back("w");
    commands_.push_back("move_backward");
    commands_.push_back("s");

    get_user_input();
  }

private:
  void get_user_input()
  {
    while (stop_)
    {
      std::cout << "Waiting for command...\n";
      std::cin >> msg_;
      check_user_input();
    }
  }

  void publish()
  {
    message_.data = msg_;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_.data.c_str());
    publisher_->publish(message_);
  }

  void check_user_input()
  {
    if (msg_ == "q" || msg_ == "quit" || msg_ == "Q")
    {
      stop_ = false;
    }
    else if (std::find(commands_.begin(), commands_.end(), msg_) == commands_.end())
    {
      std::cout << "No such command" << std::endl;
    }
    else
    {
      publish();
    }
  }

  bool stop_;
  std::string msg_;
  std::vector<std::string> commands_;
  std_msgs::msg::String message_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
