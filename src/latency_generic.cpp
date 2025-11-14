#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <cstdio>
#include <rosx_introspection/ros_parser.hpp>


using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "laserscan", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
    {
      
      rclcpp::Time reception_time = rclcpp::Time(msg->header.stamp);
      rclcpp::Time diff = this->now() - reception_time;
      double delta_ms = diff.seconds()*1000; ///1'000'000;
      // msg->header.frame_id.c_str()
      RCLCPP_INFO(this->get_logger(), "Delta: %f ms", delta_ms );
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}