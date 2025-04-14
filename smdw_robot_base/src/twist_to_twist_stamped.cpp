#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class TwistToTwistStamped : public rclcpp::Node
{
public:
  TwistToTwistStamped() : Node("twist_to_twist_stamped")
  {
    this->declare_parameter<std::string>("input_topic", "/cmd_vel");
    this->declare_parameter<std::string>("output_topic", "/mecanum_drive_controller/reference");
    this->declare_parameter<std::string>("frame_id", "base_link");

    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        input_topic, 10,
        std::bind(&TwistToTwistStamped::twist_callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(output_topic, 10);
  }

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    geometry_msgs::msg::TwistStamped twist_stamped;
    twist_stamped.header.stamp = this->now();
    twist_stamped.header.frame_id = frame_id_;
    twist_stamped.twist = *msg;
    pub_->publish(twist_stamped);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  std::string frame_id_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToTwistStamped>());
  rclcpp::shutdown();
  return 0;
}