#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      get_params();
      std::chrono::milliseconds timer_period{ (int) msg_period_ms_ };

      scan_time_s_ = msg_period_ms_*1e-3;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f', %i", scan_time_s_, msg_type_);
      switch (msg_type_)
      {
      case 0:
       
        RCLCPP_INFO(this->get_logger(), "Publishing: laserscan: %i points @ %f ms ≈ %f Hz ", scan_width_, scan_time_s_*1e-3 ,1/scan_time_s_); 
        publisher_ls_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laserscan", rclcpp::SensorDataQoS());
        timer_ = this->create_wall_timer(timer_period, std::bind(&MinimalPublisher::timer_callback_ls, this));
        
        break;
      case 1:
        RCLCPP_INFO(this->get_logger(), "Publishing: pc2: %i x %i @ %f ms ≈ %f Hz ", scan_width_, scan_height_, scan_time_s_*1e3 ,1/scan_time_s_);

        publisher_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar", rclcpp::SensorDataQoS());
        timer_ = this->create_wall_timer(timer_period, std::bind(&MinimalPublisher::timer_callback_pc, this));
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "msg_type incorrect: use only 0 or 1");
        rclcpp::shutdown();
        break;
      }

      
    }

  private:
    void get_params(){
      this->declare_parameter("use_smart_pointer", true);
      use_smart_pointer_ = this->get_parameter("use_smart_pointer").as_bool();

      this->declare_parameter("msg_period", 100.0);
      msg_period_ms_ = this->get_parameter("msg_period").as_double();

      this->declare_parameter("scan_width", 100);
      scan_width_ = this->get_parameter("scan_width").as_int();

      this->declare_parameter("scan_height", 100);
      scan_height_ = this->get_parameter("scan_height").as_int();

      this->declare_parameter("msg_type", 0);
      msg_type_ = this->get_parameter("msg_type").as_int();

      switch (msg_type_)
      {
      case 0:
        RCLCPP_INFO(this->get_logger(), "Using laserscan source");
        break;
      case 1:
        RCLCPP_INFO(this->get_logger(), "Using pointcloud source");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "msg_type incorrect: use only 0 or 1");
        rclcpp::shutdown();
        break;
      }

    }

    std::vector<float> get_random_vector(size_t n, int seed)
    {
        int8_t p_rng = seed_memory_ * seed % 2;
        std::vector<float> v(n, 0); 
        for (auto it = begin (v); it != end (v); ++it) {
            *it = p_rng;
            p_rng++;
            p_rng=(p_rng*seed)%2342;
        }

        seed_memory_++;
        return v;
    }

    std::vector<unsigned char> get_random_pc(size_t n, int seed)
    {
        int p_rng = seed_memory_ * seed % 2;
        std::vector<unsigned char> v(n, 0); 
        for (auto it = begin (v); it != end (v); ++it) {
            *it = p_rng%255;
            p_rng++;
            p_rng=(p_rng*seed)%2342;
        }

        seed_memory_++;
        return v;
    }
    
    void timer_callback_pc()
    {
        std::string frame_name = "dummy";
        std::vector payload = get_random_pc((size_t) scan_width_*scan_height_,324);
        if (use_smart_pointer_){
            auto message = std::make_unique<sensor_msgs::msg::PointCloud2>();
            message->data=payload;
            message->height = scan_height_;
            message->width = scan_width_;
            message->header.frame_id = frame_name;
            message->header.stamp = this->now();
            publisher_pc_->publish(std::move(message));
        } else {
            auto message = sensor_msgs::msg::PointCloud2();
            message.data=payload;
            message.header.frame_id = frame_name;

            message.header.stamp = this->now();
            publisher_pc_->publish(message);
        }
      
      
    }

    void timer_callback_ls()
    {   
        std::string frame_name = "dummy";
        std::vector payload_ranges = get_random_vector(scan_width_,324);
        std::vector payload_intensities = get_random_vector(scan_width_,2311);

        if (use_smart_pointer_){
            auto message = std::make_unique<sensor_msgs::msg::LaserScan>();
            message->ranges=payload_ranges;
            message->intensities=payload_intensities;
            message->scan_time = scan_time_s_;
            message->header.stamp = this->now();
            message->header.frame_id = frame_name;
            publisher_ls_->publish(std::move(message));
        } else {
            auto message = sensor_msgs::msg::LaserScan();
            message.ranges=payload_ranges;
            message.intensities=payload_intensities;
            message.scan_time = scan_time_s_;
            message.header.stamp = this->now();
            message.header.frame_id = frame_name;
            publisher_ls_->publish(message);
        }
     
    }


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_ls_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pc_;
    size_t count_;
    double msg_period_ms_;
    double scan_time_s_;
    bool use_smart_pointer_;
    int seed_memory_ = 342;
    int scan_height_, scan_width_;
    int msg_type_;

  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
  }
