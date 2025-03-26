#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MyNode1 : public rclcpp::Node
{
public:
  MyNode1()
  : Node("my_node_1"), count_(0)
  {
    this->declare_parameter("max_voltage", 42.0);
    this->declare_parameter("min_voltage", 32.0);
  
    publisher_ = this->create_publisher<std_msgs::msg::String>("node_name", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MyNode1::timer_callback, this));
    
    subscription_ = this->create_subscription<std_msgs::msg::Float32>(
    "battery_voltage", 10,
    std::bind(&MyNode1::battery_callback, this, std::placeholders::_1));
    
    publisher_bat = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "my_node_1";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  
  void battery_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    float max_voltage = this->get_parameter("max_voltage").as_double();
    float min_voltage = this->get_parameter("min_voltage").as_double();
  
  
    float voltage = msg->data;
    float percentage = (voltage - min_voltage) / (max_voltage - min_voltage) * 100.0f;
    percentage = std::clamp(percentage, 0.0f, 100.0f);
    
    auto out_msg = std_msgs::msg::Float32();
    out_msg.data = percentage;
    
    RCLCPP_INFO(this->get_logger(), "Battery Percentage: %.2f%%", percentage);
    publisher_bat->publish(out_msg);
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_bat;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode1>());
  rclcpp::shutdown();
  return 0;
}
