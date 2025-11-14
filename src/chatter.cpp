#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      scan_time_s_ = scan_time_.count()*1e-3;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", scan_time_s_);

      publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laserscan", rclcpp::SensorDataQoS());
      timer_ = this->create_wall_timer(scan_time_, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    std::vector<float> get_random_vector(size_t n, int seed)
    {
        int seconds = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        seconds+=seed;
        std::vector<float> v(n, 0); // choosesn by fair dice roll, guaranteed to be random

        for (auto it = begin (v); it != end (v); ++it) {
            *it = seconds;
            seconds++;
            seconds=(seconds*seed)%2342;
        }

        
        return v;
    }
    
    void timer_callback()
    {
        bool use_smart_pointer = false;

        if (use_smart_pointer){
            auto message = std::make_unique<sensor_msgs::msg::LaserScan>();
            message->ranges=get_random_vector(number_of_points,3434);
            message->intensities=get_random_vector(number_of_points,121);
            message->scan_time = scan_time_s_;
            message->header.stamp = this->now();
            message->header.frame_id = "dummy";
            publisher_->publish(std::move(message));
        } else {
            auto message = sensor_msgs::msg::LaserScan();
            message.ranges=get_random_vector(number_of_points,3434);
            message.intensities=get_random_vector(number_of_points,121);
            message.scan_time = scan_time_s_;
            message.header.stamp = this->now();
            message.header.frame_id = "dummy";
            publisher_->publish(message);
        }
      
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    size_t count_;
    size_t number_of_points = 36500;
    std::chrono::milliseconds scan_time_{50};
    float scan_time_s_;

    

  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
  }
