#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ugv_teleop/joy_axis_selector_logic.hpp"

namespace ugv_teleop
{

class JoyAxisSelectorNode : public rclcpp::Node
{
public:
  JoyAxisSelectorNode()
  : rclcpp::Node("joy_axis_selector_node")
  {
    config_.dpad_x_axis = this->declare_parameter<int>("dpad_x_axis", 7);
    config_.dpad_y_axis = this->declare_parameter<int>("dpad_y_axis", 6);
    config_.stick_x_axis = this->declare_parameter<int>("stick_x_axis", 1);
    config_.stick_y_axis = this->declare_parameter<int>("stick_y_axis", 0);
    config_.deadzone = this->declare_parameter<double>("deadzone", 0.1);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/ugv/joy_raw",
      10,
      [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
        auto output = apply_axis_selection(*msg, config_);
        joy_pub_->publish(output);
      });

    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/ugv/joy", 10);
    RCLCPP_INFO(this->get_logger(), "Joy selector started");
  }

private:
  JoyAxisSelectorConfig config_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ugv_teleop::JoyAxisSelectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
